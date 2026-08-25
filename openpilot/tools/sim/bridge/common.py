import os
import signal
import threading
import functools
import math
import time
import numpy as np

from collections import namedtuple
from enum import Enum
from multiprocessing import Process, Queue, Value
from abc import ABC, abstractmethod
from opendbc.car.honda.values import CruiseButtons
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.tools.sim.lib.params import set_params_enabled
from openpilot.tools.sim.lib.common import SimulatorState, World
from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors

QueueMessage = namedtuple("QueueMessage", ["type", "info"], defaults=[None])

class QueueMessageType(Enum):
  START_STATUS = 0
  CONTROL_COMMAND = 1
  TERMINATION_INFO = 2
  CLOSE_STATUS = 3

def control_cmd_gen(cmd: str):
  return QueueMessage(QueueMessageType.CONTROL_COMMAND, cmd)

def rk_loop(function, hz, exit_event: threading.Event):
  rk = Ratekeeper(hz, None)
  while not exit_event.is_set():
    function()
    rk.keep_time()


class SimulatorBridge(ABC):
  TICKS_PER_FRAME = 5
  THROTTLE_ACCEL = 1.6
  BRAKE_DECEL = 4.0
  longitudinal_accel_kp = 0.0
  longitudinal_accel_ki = 0.0
  longitudinal_accel_integral_limit = 0.0
  longitudinal_overspeed_margin = math.inf
  stock_cruise_emulation = False
  use_controller_steering_output = False

  def __init__(self, dual_camera, high_quality):
    set_params_enabled()
    self.params = Params()
    self.params.put_bool("AlphaLongitudinalEnabled", getattr(self, "alpha_longitudinal_enabled", True), block=True)
    experimental_mode = getattr(self, "experimental_mode_enabled", None)
    if experimental_mode is not None:
      self.params.put_bool("ExperimentalMode", experimental_mode, block=True)
      if experimental_mode:
        self.params.put_bool("ExperimentalModeConfirmed", True, block=True)

    self.rk = Ratekeeper(100, None)

    self.dual_camera = dual_camera
    self.high_quality = high_quality

    self._exit_event: threading.Event | None = None
    self._threads = []
    self._keep_alive = True
    self.started = Value('i', False)
    signal.signal(signal.SIGTERM, self._on_shutdown)
    self.simulator_state = SimulatorState()

    self.world: World | None = None

    self.past_startup_engaged = False
    self.startup_button_prev = True
    self.engaged_frame_count = 0
    self.manual_throttle = 0.0
    self.manual_steer = 0.0
    self.manual_brake = 0.0
    self.manual_control_deadline = 0.0
    self.stock_cruise_rearm_frames = 0
    self.last_controls = (0.0, 0.0, 0.0)
    self.openpilot_longitudinal = False
    self.longitudinal_accel_integral = 0.0

    self.test_run = False
    # SIM_HOLD_START=1 parks the car until 'h' is pressed, so a scene can be
    # inspected/captured before anything moves. Off by default.
    self.hold_car = os.environ.get("SIM_HOLD_START", "0") != "0"

  def reset_engagement(self):
    self.past_startup_engaged = False
    self.engaged_frame_count = 0
    self.stock_cruise_rearm_frames = 50
    self.longitudinal_accel_integral = 0.0

  def _on_shutdown(self, signal, frame):
    self.shutdown()

  def shutdown(self):
    self._keep_alive = False

  def bridge_keep_alive(self, q: Queue, retries: int):
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
    selfdrive_state = self.simulated_car.sm['selfdriveState']
    print(
    f"""
State:
Ignition: {self.simulator_state.ignition} Engaged: {self.simulator_state.is_engaged} Speed: {self.simulator_state.speed:.2f} m/s
Controls: steer={self.last_controls[0]:.2f} throttle={self.last_controls[1]:.2f} brake={self.last_controls[2]:.2f} OP-long={self.openpilot_longitudinal}
Openpilot: engageable={selfdrive_state.engageable} alert={selfdrive_state.alertType!r}
    """)

  @abstractmethod
  def spawn_world(self, q: Queue, /) -> World:
    pass

  def _run(self, q: Queue):
    self.world = self.spawn_world(q)

    simulated_car_class = getattr(self, "simulated_car_class", SimulatedCar)
    self.simulated_car = simulated_car_class()
    self.simulated_sensors = SimulatedSensors(self.dual_camera)

    self._exit_event = threading.Event()

    self.simulated_car_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_car.update, self.simulator_state),
                                                                        100, self._exit_event))
    self.simulated_car_thread.start()

    self.simulated_camera_thread = threading.Thread(target=rk_loop, args=(functools.partial(self.simulated_sensors.send_camera_images, self.world),
                                                                        20, self._exit_event))
    self.simulated_camera_thread.start()

    # Simulation tends to be slow in the initial steps. This prevents lagging later
    for _ in range(20):
      self.world.tick()

    # Publish stationary CAN/camera data first so manager, card, modeld,
    # controlsd, selfdrived, and the UI can initialize before the simulated car
    # enables cruise or begins moving.
    openpilot_ready_time = time.monotonic() + getattr(self, "openpilot_startup_delay", 0.0)
    openpilot_started = not self.stock_cruise_emulation

    while self._keep_alive:
      throttle_out = steer_out = brake_out = 0.0
      throttle_op = steer_op = brake_op = 0.0

      self.simulator_state.cruise_button = 0
      self.simulator_state.left_blinker = False
      self.simulator_state.right_blinker = False

      manual_control_active = time.monotonic() < self.manual_control_deadline
      if manual_control_active:
        throttle_manual = self.manual_throttle
        steer_manual = self.manual_steer
        brake_manual = self.manual_brake
      else:
        throttle_manual = steer_manual = brake_manual = 0.
        self.manual_throttle = self.manual_steer = self.manual_brake = 0.

      # Read manual controls
      if not q.empty():
        message = q.get()
        if message.type == QueueMessageType.CONTROL_COMMAND:
          m = message.info.split('_')
          if m[0] == "steer":
            steer_manual = float(m[1])
            self.manual_steer = steer_manual
            self.manual_control_deadline = time.monotonic() + 0.25
          elif m[0] == "throttle":
            throttle_manual = float(m[1])
            brake_manual = 0.0
            self.manual_throttle = throttle_manual
            self.manual_brake = 0.0
            self.manual_control_deadline = time.monotonic() + 0.25
          elif m[0] == "brake":
            brake_manual = float(m[1])
            throttle_manual = 0.0
            self.manual_brake = brake_manual
            self.manual_throttle = 0.0
            self.manual_control_deadline = time.monotonic() + 0.25
          elif m[0] == "cruise":
            if m[1] == "down":
              self.simulator_state.cruise_button = CruiseButtons.DECEL_SET
              if not self.simulator_state.is_engaged:
                self.stock_cruise_rearm_frames = 50
            elif m[1] == "up":
              self.simulator_state.cruise_button = CruiseButtons.RES_ACCEL
            elif m[1] == "cancel":
              self.simulator_state.cruise_button = CruiseButtons.CANCEL
            elif m[1] == "main":
              self.simulator_state.cruise_button = CruiseButtons.MAIN
          elif m[0] == "blinker":
            if m[1] == "left":
              self.simulator_state.left_blinker = True
            elif m[1] == "right":
              self.simulator_state.right_blinker = True
          elif m[0] == "ignition":
            self.simulator_state.ignition = not self.simulator_state.ignition
          elif m[0] == "hold":
            self.hold_car = not self.hold_car
            print(f"\n*** car {'HELD (press h to release)' if self.hold_car else 'RELEASED'} ***\n")
          elif m[0] == "reset":
            self.world.reset()
            self.reset_engagement()
          elif m[0] == "quit":
            break

      self.simulator_state.user_brake = brake_manual
      self.simulator_state.user_gas = throttle_manual
      self.simulator_state.user_torque = steer_manual * -10000

      steer_manual = steer_manual * -40

      # Update openpilot on current sensor state
      self.simulated_sensors.update(self.simulator_state, self.world)

      self.simulated_car.sm.update(0)
      if not openpilot_started:
        openpilot_started = (time.monotonic() >= openpilot_ready_time and
                             self.simulated_car.sm.alive['selfdriveState'])
        if openpilot_started:
          self.world.set_openpilot_ready()
      openpilot_startup_ready = openpilot_started
      self.simulator_state.is_engaged = self.simulated_car.sm['selfdriveState'].active
      if self.stock_cruise_emulation:
        # Stock-ACC cars engage from the vehicle cruise enabled edge. Pulse that
        # edge until selfdrived accepts it, then hold it enabled.
        if openpilot_startup_ready:
          if self.stock_cruise_rearm_frames > 0:
            # Force stock ACC off briefly. Re-enabling it after this interval
            # creates the PCM edge selfdrived needs after a crash or world reset.
            self.simulator_state.stock_cruise_enabled = False
            self.stock_cruise_rearm_frames -= 1
          else:
            self.simulator_state.stock_cruise_enabled = (self.simulator_state.is_engaged or self.past_startup_engaged or
                                                         (self.rk.frame // 50) % 2 == 1)
        else:
          self.simulator_state.stock_cruise_enabled = False

      if self.simulator_state.is_engaged:
        self.openpilot_longitudinal = (self.stock_cruise_emulation and
                                       self.simulated_car.sm['carParams'].openpilotLongitudinalControl)
        if self.openpilot_longitudinal:
          requested_accel = self.simulated_car.sm['carControl'].actuators.accel
          actual_accel = self.simulated_car.sm['carState'].aEgo
          accel_error = requested_accel - actual_accel
          if manual_control_active:
            self.longitudinal_accel_integral = 0.0
          else:
            self.longitudinal_accel_integral = np.clip(
              self.longitudinal_accel_integral + self.longitudinal_accel_ki * accel_error / 100.0,
              -self.longitudinal_accel_integral_limit,
              self.longitudinal_accel_integral_limit,
            )

          # Close the loop around measured acceleration. The integral learns
          # CARLA's rolling/engine resistance without creating a throttle term
          # that grows unbounded with speed.
          actuator_accel = (requested_accel + self.longitudinal_accel_kp * accel_error +
                            self.longitudinal_accel_integral)

          # The simulated cruise speed is a hard upper bound for positive
          # actuation. This guard is independent of the planner so a stale
          # positive accel request can never run the CARLA vehicle away again.
          cruise_speed = getattr(self.simulator_state, 'cruise_speed', None)
          if cruise_speed is not None and self.simulator_state.speed > cruise_speed + self.longitudinal_overspeed_margin:
            actuator_accel = min(actuator_accel, 0.0)
            self.longitudinal_accel_integral = min(self.longitudinal_accel_integral, 0.0)

          if actuator_accel >= 0.0:
            throttle_op = np.clip(actuator_accel / self.THROTTLE_ACCEL, 0.0, 1.0)
            brake_op = 0.0
          else:
            throttle_op = 0.0
            brake_op = np.clip(-actuator_accel / self.BRAKE_DECEL, 0.0, 1.0)
        else:
          if self.stock_cruise_emulation:
            # This simulated Tesla uses stock longitudinal control. In that mode
            # carControl.actuators.accel is an inactive sentinel (often -3.5), not
            # a brake request. Model the missing stock ACC with a small speed loop
            # while openpilot supplies lateral control.
            speed_error = getattr(self, "stock_cruise_speed", 8.0) - self.simulator_state.speed
            throttle_op = np.clip(speed_error * 0.18, 0.0, 0.5)
            brake_op = np.clip(-speed_error * 0.12, 0.0, 0.4)
          else:
            # Preserve the upstream MetaDrive actuator mapping.
            requested_accel = self.simulated_car.sm['carControl'].actuators.accel
            throttle_op = np.clip(requested_accel / self.THROTTLE_ACCEL, 0.0, 1.0)
            brake_op = np.clip(-requested_accel / self.BRAKE_DECEL, 0.0, 1.0)
        # Actuate what the car's own controller would put on the bus, not the
        # raw request. carControl.actuators is the controller's unbounded
        # feedforward; the brand carcontroller then applies lateral accel/jerk
        # limits, MAX_ANGLE_RATE and the EPS angle ceiling before anything
        # reaches the rack. Reading the request directly let angles well past
        # the EPS fault threshold hit the simulated car in a single step.
        if self.use_controller_steering_output:
          steer_op = self.simulated_car.sm['carOutput'].actuatorsOutput.steeringAngleDeg
        else:
          steer_op = self.simulated_car.sm['carControl'].actuators.steeringAngleDeg

        if self.stock_cruise_emulation:
          self.engaged_frame_count += 1
        if not self.stock_cruise_emulation or self.engaged_frame_count >= 300:
          self.past_startup_engaged = True
      elif self.stock_cruise_emulation and not self.past_startup_engaged and openpilot_startup_ready and not self.hold_car:
        self.engaged_frame_count = 0
        # Hold each phase long enough for the independent 100 Hz CAN thread to
        # observe it reliably. Keep trying while controls initialize since
        # engageable can be true for intervals shorter than a CAN phase.
        self.simulator_state.cruise_button = CruiseButtons.DECEL_SET if (self.rk.frame // 20) % 2 == 0 else CruiseButtons.MAIN
      elif not self.past_startup_engaged and self.simulated_car.sm['selfdriveState'].engageable:
        self.simulator_state.cruise_button = CruiseButtons.DECEL_SET if self.startup_button_prev else CruiseButtons.MAIN
        self.startup_button_prev = not self.startup_button_prev

      startup_throttle = (getattr(self, "startup_throttle", 0.0)
                          if openpilot_startup_ready and not self.past_startup_engaged
                          and not self.hold_car else 0.0)
      # Keyboard/joystick input is an explicit simulator safety override. Keep
      # it available even while openpilot is engaged so a user can always move,
      # brake, or steer the ego vehicle from the bridge terminal.
      if self.hold_car and not manual_control_active:
        # Park it: brake hard and command nothing, so the scene can be
        # inspected before anything moves. Manual keys still override.
        throttle_out = 0.0
        brake_out = 1.0
        steer_out = 0.0
      elif manual_control_active:
        throttle_out = throttle_manual
        brake_out = brake_manual
        steer_out = steer_manual
      elif self.simulator_state.is_engaged:
        throttle_out = throttle_op
        brake_out = brake_op
        steer_out = steer_op
      else:
        throttle_out = max(throttle_manual, startup_throttle)
        brake_out = brake_manual
        steer_out = steer_manual

      self.last_controls = (float(steer_out), float(throttle_out), float(brake_out))
      if brake_out > 0.01 and hasattr(self, "brake_seen"):
        self.brake_seen.value = True
      self.world.apply_controls(steer_out, throttle_out, brake_out)
      self.world.read_state()
      self.world.read_sensors(self.simulator_state)

      if self.world.consume_reset_event():
        self.reset_engagement()

      if self.world.exit_event.is_set():
        self.shutdown()

      if self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.world.tick()
        self.world.read_cameras()

      # don't print during test, so no print/IO Block between OP and metadrive processes
      if not self.test_run and self.rk.frame % 25 == 0:
        self.print_status()

      self.started.value = True

      self.rk.keep_time()
