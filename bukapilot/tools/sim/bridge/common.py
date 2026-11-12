"""Simulator-agnostic bridge loop, ported from openpilot/tools/sim/bridge/common.py
to bukapilot (openpilot 0.8.13):

  * engagement state comes from controlsState.active/engageable (there is no
    selfdriveState in 0.8.13);
  * steering follows carControl.actuators.steeringAngleDeg with the legacy
    bridge's rate limit (there is no carOutput in 0.8.13);
  * the Honda Civic is openpilot-longitudinal (comma pedal), so throttle and
    brake are always derived from carControl.actuators.accel, closed around
    carState.aEgo with a small PI loop and capped at the cruise set speed.
"""
import functools
import signal
import threading
import time
from abc import ABC, abstractmethod
from collections import namedtuple
from enum import Enum
from multiprocessing import Process, Value

import numpy as np

import cereal.messaging as messaging
from cereal import log
from common.realtime import Ratekeeper
from selfdrive.car.honda.values import CruiseButtons
from tools.sim.lib.common import SimulatorState
from tools.sim.lib.params import set_params_enabled

QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])


class QueueMessageType(Enum):
  START_STATUS = 0
  CONTROL_COMMAND = 1
  TERMINATION_INFO = 2
  CLOSE_STATUS = 3


def control_cmd_gen(cmd: str):
  return QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd)


def toggle_blinker(current, requested):
  """z / x latch a blinker on; pressing the same key again turns it off, the other key switches side."""
  return None if current == requested else requested


def plan_manual_routing(manual_active, steer, throttle, brake, blinker, engaged, lane_change_ready):
  """Which parts of a manual key press override openpilot: (pedals_manual, steering_manual).

  A steer key in the direction of the blinker is only a nudge (steering torque goes to the car, openpilot
  keeps steering) while the car is engaged and openpilot can act on it, that is `lane_change_ready`: the
  lane change state is not `off`. Steer keys: a = -0.15 (left), d = +0.15.
  In every other case a key press overrides everything, so the user can always steer by hand.
  """
  nudge = (manual_active and engaged and lane_change_ready and steer != 0 and
           ((blinker == "left" and steer < 0) or (blinker == "right" and steer > 0)))
  if nudge:
    return (throttle != 0 or brake != 0), False
  return manual_active, manual_active


LONG_PRESS_FRAMES = 55    # controlsd adds 5 km/h after 50 frames of a held button
SHORT_PRESS_FRAMES = 10   # one press adds 1 km/h
CRUISE_PRESS_GAP = 25     # idle frames between presses


def next_cruise_press(v_cruise, target):
  """Frames to hold RES+ to move the set speed toward `target` (long press +5, short press +1), or None."""
  if target - v_cruise >= 5:
    return LONG_PRESS_FRAMES
  if target - v_cruise >= 1:
    return SHORT_PRESS_FRAMES
  return None


def rk_loop(function, hz, exit_event: threading.Event):
  rk = Ratekeeper(hz, None)
  while not exit_event.is_set():
    function()
    rk.keep_time()


def rate_limit(old, new, limit):
  return float(np.clip(new, old - limit, old + limit))


class SimulatorBridge(ABC):
  TICKS_PER_FRAME = 5           # 100 Hz loop, 20 Hz simulator/camera
  THROTTLE_ACCEL = 1.6          # m/s^2 at full throttle
  BRAKE_DECEL = 4.0             # m/s^2 at full brake
  longitudinal_accel_kp = 0.0
  longitudinal_accel_ki = 0.0
  longitudinal_accel_integral_limit = 0.0
  longitudinal_overspeed_margin = 0.5  # m/s above vCruise where positive accel is cut
  STEER_RATE_LIMIT = 0.5        # deg per 10 ms step, same as the legacy bridge
  MANUAL_STEER_DEG = -100.0     # steering wheel deg per unit of keyboard/joystick steer
  MANUAL_TORQUE = -10000.0      # STEER_TORQUE_SENSOR per unit, > STEER_THRESHOLD(1200)
  MANUAL_TIMEOUT = 0.25         # s a manual input stays applied after the last key event
  HELD_INPUT_FRAMES = 10        # frames a cruise button / blinker key stays pressed

  def __init__(self, auto_engage=True, test_run=False, cruise_kph=None):
    set_params_enabled()

    self.rk = Ratekeeper(100, None)
    self.auto_engage = auto_engage
    self.test_run = test_run

    self._exit_event = None
    self._keep_alive = True
    self.started = Value('i', False)
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self.simulator_state = SimulatorState()
    self.world = None

    self.past_startup_engaged = False
    self.manual_throttle = self.manual_steer = self.manual_brake = 0.0
    self.manual_control_deadline = 0.0
    self.last_controls = (0.0, 0.0, 0.0)
    self.last_steer = 0.0
    self.longitudinal_accel_integral = 0.0
    self.hold_car = False
    # Key presses are single events; hold them on the bus long enough that the
    # independent 100 Hz CAN thread and carstate both see them.
    self.held_button = 0
    self.held_frames = 0
    # one-shot: after engaging, raise the set speed to this value (the engage default is only 40 km/h)
    self.cruise_kph_pending = cruise_kph
    self.cruise_press_wait = 0
    # z / x latch the blinker (ALC needs it to stay on); only the user turns it off again
    self.blinker = None

  def reset_engagement(self):
    self.past_startup_engaged = False
    self.longitudinal_accel_integral = 0.0
    self.last_steer = 0.0

  def _on_shutdown(self, signum, frame):
    self.shutdown()

  def shutdown(self):
    self._keep_alive = False

  def bridge_keep_alive(self, q, retries):
    try:
      self._run(q)
    finally:
      self.close("bridge terminated")

  def close(self, reason):
    self.started.value = False
    if self._exit_event is not None:
      self._exit_event.set()
    if self.world is not None:
      self.world.close(reason)

  def run(self, queue, retries=-1):
    bridge_p = Process(name="bridge", target=self.bridge_keep_alive, args=(queue, retries))
    bridge_p.start()
    return bridge_p

  def print_status(self):
    cs = self.sm['controlsState']
    s = self.simulator_state
    print(f"engaged={s.is_engaged} engageable={cs.engageable} speed={s.speed:.2f} m/s "
          f"vCruise={cs.vCruise:.0f} kph steer={self.last_controls[0]:+.1f} throttle={self.last_controls[1]:.2f} "
          f"brake={self.last_controls[2]:.2f} alert={cs.alertType!r}", flush=True)

  @abstractmethod
  def spawn_world(self, q):
    pass

  def _handle_command(self, cmd, s):
    """Apply one keyboard/joystick command. Returns False to quit."""
    m = cmd.split('_')
    now = time.monotonic()
    if m[0] == "steer":
      self.manual_steer = float(m[1])
      self.manual_control_deadline = now + self.MANUAL_TIMEOUT
    elif m[0] == "throttle":
      self.manual_throttle, self.manual_brake = float(m[1]), 0.0
      self.manual_control_deadline = now + self.MANUAL_TIMEOUT
    elif m[0] == "brake":
      self.manual_brake, self.manual_throttle = float(m[1]), 0.0
      self.manual_control_deadline = now + self.MANUAL_TIMEOUT
    elif m[0] == "cruise":
      self.held_button = {"down": CruiseButtons.DECEL_SET, "up": CruiseButtons.RES_ACCEL,
                          "cancel": CruiseButtons.CANCEL, "main": CruiseButtons.MAIN}.get(m[1], 0)
      self.held_frames = self.HELD_INPUT_FRAMES
      self.cruise_kph_pending = None  # a manual cruise key takes over from the automatic ramp
      if m[1] == "cancel":
        # an explicit cancel also stops auto-engage from re-engaging right away
        self.past_startup_engaged = True
    elif m[0] == "blinker":
      self.blinker = toggle_blinker(self.blinker, m[1])
    elif m[0] == "ignition":
      s.ignition = not s.ignition
    elif m[0] == "hold":
      self.hold_car = not self.hold_car
      print(f"\n*** car {'HELD (press h to release)' if self.hold_car else 'RELEASED'} ***\n")
    elif m[0] == "reset":
      self.world.reset()
      self.reset_engagement()
      self.blinker = None
    elif m[0] == "quit":
      return False
    return True

  def _openpilot_longitudinal(self, manual_control_active):
    requested_accel = self.sm['carControl'].actuators.accel
    accel_error = requested_accel - self.sm['carState'].aEgo
    if manual_control_active:
      self.longitudinal_accel_integral = 0.0
    else:
      self.longitudinal_accel_integral = float(np.clip(
        self.longitudinal_accel_integral + self.longitudinal_accel_ki * accel_error / 100.0,
        -self.longitudinal_accel_integral_limit, self.longitudinal_accel_integral_limit))
    accel = requested_accel + self.longitudinal_accel_kp * accel_error + self.longitudinal_accel_integral

    # Hard upper bound on positive actuation: never exceed the set speed.
    v_cruise = self.sm['controlsState'].vCruise / 3.6
    if self.simulator_state.speed > v_cruise + self.longitudinal_overspeed_margin:
      accel = min(accel, 0.0)
      self.longitudinal_accel_integral = min(self.longitudinal_accel_integral, 0.0)

    if accel >= 0.0:
      return float(np.clip(accel / self.THROTTLE_ACCEL, 0.0, 1.0)), 0.0
    return 0.0, float(np.clip(-accel / self.BRAKE_DECEL, 0.0, 1.0))

  def _run(self, q):
    # imported here so the bridge module can be loaded without the sim deps
    from tools.sim.lib.simulated_car import SimulatedCar
    from tools.sim.lib.simulated_sensors import SimulatedSensors

    self.world = self.spawn_world(q)
    self.simulated_car = SimulatedCar()
    self.simulated_sensors = SimulatedSensors()
    # The car thread has its own SubMaster; never share one across threads.
    self.sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'lateralPlan'])

    self._exit_event = threading.Event()
    threading.Thread(target=rk_loop, daemon=True,
                     args=(functools.partial(self.simulated_car.update, self.simulator_state), 100, self._exit_event)).start()
    threading.Thread(target=rk_loop, daemon=True,
                     args=(functools.partial(self.simulated_sensors.send_camera_images, self.world), 20, self._exit_event)).start()

    # simulation is slow in the first steps; get them out of the way
    for _ in range(20):
      self.world.tick()

    s = self.simulator_state
    while self._keep_alive:
      s.cruise_button = 0
      s.left_blinker = s.right_blinker = False

      manual_control_active = time.monotonic() < self.manual_control_deadline
      if not manual_control_active:
        self.manual_throttle = self.manual_steer = self.manual_brake = 0.0

      if not q.empty():
        message = q.get()
        if message.type == QueueMessageType.CONTROL_COMMAND and not self._handle_command(message.info, s):
          break
        manual_control_active = time.monotonic() < self.manual_control_deadline

      if self.held_frames > 0:
        self.held_frames -= 1
        s.cruise_button = self.held_button
      else:
        self.held_button = 0

      self.sm.update(0)
      lane_change_ready = self.sm['lateralPlan'].laneChangeState != log.LateralPlan.LaneChangeState.off
      s.left_blinker = self.blinker == "left"
      s.right_blinker = self.blinker == "right"

      s.user_brake = self.manual_brake
      s.user_gas = self.manual_throttle
      s.user_torque = self.manual_steer * self.MANUAL_TORQUE

      self.simulated_sensors.update(s)

      cs = self.sm['controlsState']
      s.is_engaged = cs.active
      pedals_manual, steer_manual = plan_manual_routing(manual_control_active, self.manual_steer, self.manual_throttle,
                                                        self.manual_brake, self.blinker, s.is_engaged,
                                                        lane_change_ready)

      if self.cruise_press_wait > 0:
        self.cruise_press_wait -= 1
      elif self.cruise_kph_pending and s.is_engaged and self.held_frames == 0:
        frames = next_cruise_press(cs.vCruise, self.cruise_kph_pending)
        if frames is None:
          self.cruise_kph_pending = None
        else:
          self.held_button, self.held_frames = CruiseButtons.RES_ACCEL, frames
          self.cruise_press_wait = frames + CRUISE_PRESS_GAP

      throttle_op = brake_op = 0.0
      steer_op = 0.0
      if s.is_engaged:
        throttle_op, brake_op = self._openpilot_longitudinal(pedals_manual)
        steer_op = rate_limit(self.last_steer, self.sm['carControl'].actuators.steeringAngleDeg, self.STEER_RATE_LIMIT)
        self.past_startup_engaged = True
      elif (self.auto_engage and not self.past_startup_engaged and cs.engageable and not self.hold_car
            and self.held_frames == 0):
        # Honda non-PCM cruise engages on the SET button *release*. Hold each
        # phase for 20 frames so the independent 100 Hz CAN thread sees it.
        s.cruise_button = CruiseButtons.DECEL_SET if (self.rk.frame // 20) % 2 == 0 else 0

      if self.hold_car and not manual_control_active:
        throttle_out, brake_out, steer_out = 0.0, 1.0, 0.0
      elif manual_control_active:
        # keyboard/joystick is a simulator override and always wins, except that a steer key pressed in the
        # direction of the blinker while a lane change is ready or running is only a nudge: openpilot keeps steering
        if pedals_manual:
          throttle_out, brake_out = self.manual_throttle, self.manual_brake
        else:
          throttle_out, brake_out = throttle_op, brake_op
        steer_out = self.manual_steer * self.MANUAL_STEER_DEG if steer_manual else steer_op
      elif s.is_engaged:
        throttle_out, brake_out, steer_out = throttle_op, brake_op, steer_op
      else:
        throttle_out = brake_out = steer_out = 0.0

      self.last_steer = steer_out
      self.last_controls = (float(steer_out), float(throttle_out), float(brake_out))
      self.world.apply_controls(steer_out, throttle_out, brake_out)
      self.world.read_sensors(s)

      if self.world.consume_reset_event():
        self.reset_engagement()
      if self.world.exit_event.is_set():
        self.shutdown()

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      if self.rk.frame % 100 == 0:
        self.print_status()

      self.started.value = True
      self.rk.keep_time()
