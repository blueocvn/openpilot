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
import os
import pathlib
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
from tools.sim.bridge.run_metrics import FrameRecorder, OvertakeScript, RunMetrics
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


def blinker_for_command(requested):
  """z latches the left blinker, c the right one, x puts it back to center (off). Pressing a side key again keeps it on."""
  return None if requested == "off" else requested


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


def nearest_ahead(ego_xy, ego_forward_xy, ego_half_length, others, lateral_limit):
  """Nearest object ahead within `lateral_limit` m of the ego centre line: (bumper gap m, speed along ego heading
  m/s, lateral offset m with left positive as openpilot's yRel), or None. `others` is an iterable of
  (xy, velocity_xy, half_length), the half-length along the ego heading."""
  fx, fy = ego_forward_xy
  norm = float(np.hypot(fx, fy)) or 1.0
  fx, fy = fx / norm, fy / norm
  best = None
  for (ox, oy), (vx, vy), half_length in others:
    dx, dy = ox - ego_xy[0], oy - ego_xy[1]
    longitudinal = dx * fx + dy * fy
    lateral = dx * fy - dy * fx   # CARLA is left-handed (x forward, y right): this is positive to the left
    if longitudinal <= 0.0 or abs(lateral) > lateral_limit:
      continue
    gap = longitudinal - ego_half_length - half_length
    if best is None or gap < best[0]:
      best = (gap, vx * fx + vy * fy, lateral)
  return best


def lead_truth(ego_xy, ego_forward_xy, ego_half_length, others, lane_half_width=2.0):
  """Simulator ground truth of the nearest object ahead in the ego lane: (bumper gap m, speed along ego heading m/s).

  For logging only, to grade what the camera model reports. It is never given to openpilot: the real car has a
  camera and nothing else. `others` is an iterable of (xy, velocity_xy, half_length).
  """
  found = nearest_ahead(ego_xy, ego_forward_xy, ego_half_length, others, lane_half_width)
  return None if found is None else found[:2]


CLOSE_LEAD_LOG_M = 40.0   # log at 10 Hz while the true gap to the lead is below this
LEAD_FRAME_GAP_M = 15.0    # save the camera frame (what the model receives) while the true gap is below this
LEAD_FRAME_PERIOD_S = 0.5  # ... at most this often,
LEAD_FRAME_MAX = 200       # ... and at most this many per run
LEAD_FRAME_DIR = pathlib.Path(os.environ.get("LEAD_FRAME_DIR") or pathlib.Path(__file__).resolve().parents[3] / ".carla" / "lead-frames")
OVERTAKE_TAIL_S = 3.0      # the overtake run ends this long after the scripted driver is done
COLLISION_LOG_GAP_S = 1.0  # one collision banner per second at most (CARLA reports every tick of a sustained contact)


def frame_label(truth, seen):
  """Caption stamped on a saved camera frame: the simulator's truth next to what openpilot perceived."""
  perceived = "OP dRel={:.1f}m vLead={:.1f} prob={:.2f}".format(*seen) if seen else "OP NO LEAD"
  return f"SIM gap={truth[0]:.1f}m vLead={truth[1]:.1f}  |  {perceived}"


def frame_name(index, truth, seen):
  drel = f"{seen[0]:.1f}" if seen else "none"
  return f"{index:04d}_gap{max(truth[0], 0.0):05.1f}_op{drel}.jpg"


def save_lead_frame(bgra, path, label):
  """Write a camera frame as a JPEG with `label` stamped on a band at the top. Runs off the bridge loop."""
  from PIL import Image, ImageDraw, ImageFont
  image = Image.fromarray(bgra[:, :, [2, 1, 0]])
  draw = ImageDraw.Draw(image)
  try:
    font = ImageFont.load_default(size=40)
  except TypeError:  # Pillow < 10.1 has a fixed-size default font only
    font = ImageFont.load_default()
  draw.rectangle([0, 0, image.width, 64], fill=(0, 0, 0))
  draw.text((16, 10), label, fill=(255, 255, 0), font=font)
  path.parent.mkdir(parents=True, exist_ok=True)
  image.save(path, quality=85)


def collision_banner(other, impulse, sim_speed, seen):
  """Loud, upper-case collision notice for the lead log. `seen` is what openpilot saw at that moment."""
  perceived = ("OPENPILOT SAW LEAD: DREL={:.1f}M VLEAD={:.1f} PROB={:.2f}".format(*seen)
               if seen else "OPENPILOT SAW NO LEAD")
  bar = "#" * 78
  lines = [bar, "#####  !!!  COLLISION  !!!  COLLISION  !!!  COLLISION  !!!  #####",
           f"COLLIDED WITH: {other} | EGO SPEED: {sim_speed:.1f} M/S | IMPACT: {impulse:.0f}",
           perceived, bar]
  return "\n".join(line.upper() for line in lines)


def status_line(op, seen, sim_speed, truth):
  """One log line that keeps apart what openpilot decides/perceives and what only the simulator knows.

  [OP cmd]  openpilot's output (planner source, requested accel) and the pedals the bridge derived from it
  [OP sees] the lead openpilot perceives from the camera model: (dRel m, vLead m/s, prob) or None
  [SIM]     simulator ground truth: own speed and (bumper gap m, lead speed m/s); never given to openpilot
  [ERR]     what openpilot sees minus the truth
  """
  line = (f"[OP cmd] engaged={op['engaged']} vCruise={op['v_cruise']:.0f}kph plan={op['source']} "
          f"accel={op['accel']:+.2f} throttle={op['throttle']:.2f} brake={op['brake']:.2f} steer={op['steer']:+.1f} "
          f"fcw={op['fcw']} alert={op['alert']!r}")
  line += (" | [OP sees] lead dRel={:.1f}m vLead={:.1f} prob={:.2f}".format(*seen)
           if seen else " | [OP sees] NO LEAD")
  line += f" | [SIM] speed={sim_speed:.1f}"
  if truth is None:
    return line
  gap, v_lead = truth
  closing = sim_speed - v_lead
  ttc = f"{gap / closing:.1f}s" if closing > 0.1 and gap > 0.0 else ("CONTACT" if gap <= 0.0 else "-")
  line += f" lead gap={gap:.1f}m vLead={v_lead:.1f} closing={closing:+.1f} ttc={ttc}"
  if seen:
    line += f" | [ERR] dRel={seen[0] - gap:+.1f}m vLead={seen[1] - v_lead:+.1f}"
  else:
    line += " | [ERR] lead missed"
  return line


LONG_PRESS_FRAMES = 55   # controlsd adds 5 km/h after 50 frames of a held button
SHORT_PRESS_FRAMES = 10   # one press adds 1 km/h
CRUISE_PRESS_GAP = 25     # idle frames between presses


def next_cruise_press(v_cruise, target):
  """Frames to hold RES+ to move the set speed up toward `target` (long press +5, short press +1), or None."""
  if target - v_cruise >= 5:
    return LONG_PRESS_FRAMES
  if target - v_cruise >= 1:
    return SHORT_PRESS_FRAMES
  return None


def next_cruise_action(v_cruise, target):
  """(button, frames) that moves the set speed toward `target`, up with RES+ or down with SET-, or None when there."""
  if v_cruise < target:
    frames = next_cruise_press(v_cruise, target)
    return None if frames is None else (CruiseButtons.RES_ACCEL, frames)
  frames = next_cruise_press(-v_cruise, -target)   # the same thresholds, going down
  return None if frames is None else (CruiseButtons.DECEL_SET, frames)


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
    # z / c latch the blinker (ALC needs it to stay on); only x (center) turns it off again
    self.blinker = None
    # LEAD_LOG=1: the status line splits openpilot's view from the simulator's ground truth (10 Hz near a lead)
    self.lead_log = os.environ.get("LEAD_LOG") == "1"
    self.last_collision_print = float("-inf")
    self.last_frame_save = float("-inf")
    self.frames_saved = 0
    # Benchmark scoring, all off unless asked for (see tools/sim-linux-docker/bench.sh):
    #   LEAD_RESULT_FILE=<path>  one JSON result per run, written at exit; the run also ends by itself (collision,
    #                            stopped for good, LEAD_RUN_DISTANCE_M driven)
    #   LEAD_SCENARIO=<label>    copied into the result;  LEAD_OVERTAKE=1: a scripted driver steers out and passes
    #   LEAD_RECORD_DIR=<dir>    every LEAD_RECORD_EVERY-th camera frame (JPEG) plus truth.jsonl with ground truth
    #                            and the model's lead output; LEAD_RECORD_IMAGES=0 writes truth.jsonl only
    self.result_file = os.environ.get("LEAD_RESULT_FILE") or None
    self.overtake = OvertakeScript() if os.environ.get("LEAD_OVERTAKE") == "1" else None
    self.metrics = None
    if self.result_file:
      distance = os.environ.get("LEAD_RUN_DISTANCE_M")
      self.metrics = RunMetrics(os.environ.get("LEAD_SCENARIO", ""), float(distance) if distance else None,
                                stop_when_stopped=self.overtake is None)
    record_dir = os.environ.get("LEAD_RECORD_DIR")
    self.recorder = (FrameRecorder(record_dir, int(os.environ.get("LEAD_RECORD_EVERY") or 1),
                                   images=os.environ.get("LEAD_RECORD_IMAGES", "1") != "0") if record_dir else None)
    self.end_reason = None
    self.wall_start = None   # (wall clock, simulated clock) at the first scored sample

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

  def print_status(self, truth=None):
    cs = self.sm['controlsState']
    s = self.simulator_state
    if not self.lead_log:
      print(f"engaged={s.is_engaged} engageable={cs.engageable} speed={s.speed:.2f} m/s "
            f"vCruise={cs.vCruise:.0f} kph steer={self.last_controls[0]:+.1f} throttle={self.last_controls[1]:.2f} "
            f"brake={self.last_controls[2]:.2f} alert={cs.alertType!r}", flush=True)
      return
    plan = self.sm['longitudinalPlan']
    op = dict(engaged=bool(s.is_engaged), v_cruise=cs.vCruise, alert=cs.alertType,
              accel=self.sm['carControl'].actuators.accel, steer=self.last_controls[0],
              throttle=self.last_controls[1], brake=self.last_controls[2],
              source=plan.longitudinalPlanSource, fcw=bool(plan.fcw))
    print(status_line(op, self.seen_lead(), s.speed, truth), flush=True)

  def seen_lead(self):
    """(dRel, vLead, prob) of the lead openpilot perceives, or None."""
    lead = self.sm['radarState'].leadOne
    return (lead.dRel, lead.vLead, lead.modelProb) if lead.status else None

  def seen_y_rel(self):
    """yRel (m, left positive) of the lead openpilot perceives, or None."""
    lead = self.sm['radarState'].leadOne
    return lead.yRel if lead.status else None

  def save_frame_near_lead(self, truth):
    """While the true gap is small, keep the exact camera frame openpilot is given, labelled with truth and perception."""
    now = time.monotonic()
    if truth[0] >= LEAD_FRAME_GAP_M or now - self.last_frame_save < LEAD_FRAME_PERIOD_S or self.frames_saved >= LEAD_FRAME_MAX:
      return
    self.last_frame_save = now
    seen = self.seen_lead()
    path = LEAD_FRAME_DIR / frame_name(self.frames_saved, truth, seen)
    if self.frames_saved == 0:
      print(f"lead log: saving camera frames to {LEAD_FRAME_DIR}", flush=True)
    self.frames_saved += 1
    # the array is overwritten by the next frame: copy it, then encode in the background
    threading.Thread(target=save_lead_frame, args=(self.world.road_bgra_image.copy(), path, frame_label(truth, seen)),
                     daemon=True).start()

  def drain_collisions(self):
    """Take the ego car's collisions from the simulator: score them, and print the banner when --lead-log is on."""
    for other, impulse in self.world.consume_collisions():
      if self.metrics is not None:
        self.metrics.collide(other, self.simulator_state.speed)
      now = time.monotonic()
      if self.lead_log and now - self.last_collision_print >= COLLISION_LOG_GAP_S:
        self.last_collision_print = now
        print(collision_banner(other, impulse, self.simulator_state.speed, self.seen_lead()), flush=True)

  def sim_time(self):
    """Simulated seconds: every loop frame is 10 ms of simulation, however long it took on the wall clock.
    Scoring uses this clock, so a slow machine does not change distances or decelerations."""
    return self.rk.frame * 0.01

  def score_sample(self, truth):
    """One 10 Hz benchmark sample: ground truth next to what the car did. Measurement only."""
    cs = self.sm['controlsState']
    y_pair = None
    if self.overtake is not None and self.overtake.active:
      y_seen, y_true = self.seen_y_rel(), self.world.truth_lateral()
      y_pair = (y_seen, y_true) if y_seen is not None and y_true is not None else None
    if self.wall_start is None and self.metrics.t0 is not None:
      self.wall_start = (time.monotonic(), self.sim_time())
    self.metrics.sample(self.sim_time(), self.simulator_state.speed, truth, self.seen_lead(), cs.vCruise,
                        brake=self.last_controls[2], engaged=bool(self.simulator_state.is_engaged),
                        in_overtake=self.overtake is not None and self.overtake.active, y_rel_pair=y_pair)

  def record_frame(self):
    """Hand the camera frame openpilot was just given to the recorder, with the simulator's truth for that frame."""
    truth, lateral = self.world.truth_lead(), self.world.truth_lateral()
    seen = self.seen_lead()
    leads = self.sm['modelV2'].leadsV3
    model_lead = None
    if len(leads) > 0:   # what the camera model says before radard filters it
      lead = leads[0]
      model_lead = {"x": lead.x[0], "y": lead.y[0], "v": lead.v[0], "x_std": lead.xStd[0], "v_std": lead.vStd[0],
                    "prob": lead.prob}
    self.recorder.add(self.world.road_bgra_image, {
      "model_lead": model_lead,
      "tick": self.world.tick_count, "wall_s": time.monotonic(), "engaged": bool(self.simulator_state.is_engaged),
      "ego_speed": self.simulator_state.speed, "pose": self.world.ego_pose(),
      "truth_gap": None if truth is None else truth[0], "truth_v": None if truth is None else truth[1],
      "truth_lateral": lateral, "op_lead": None if seen is None else list(seen),
      "op_accel": float(self.sm['carControl'].actuators.accel), "v_cruise_kph": float(self.sm['controlsState'].vCruise)})

  def intended_gap(self, summary):
    """The gap the longitudinal planner aims for at the steady speeds of a run, or None without a steady follow."""
    if "steady_gap_m" not in summary:
      return None
    try:
      from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance, get_desired_tf
      tf = get_desired_tf(self.sm['carState'].cruiseState.setDistance)
      return desired_follow_distance(summary["steady_ego_speed_mps"], summary["steady_obstacle_speed_mps"], 0, tf)
    except Exception as exc:  # measurement must never break the shutdown
      print(f"intended gap unavailable: {exc}", flush=True)
      return None

  def finish_run(self):
    if self.recorder is not None:
      self.recorder.close()
    if self.metrics is not None:
      self.metrics.end_reason = self.end_reason
      extra = {"intended_gap_m": self.intended_gap(self.metrics.summary())}
      if self.wall_start is not None and time.monotonic() > self.wall_start[0]:
        # 1.0 = the simulation kept up with real time; below 1 openpilot saw the world in slow motion
        extra["realtime_factor"] = (self.sim_time() - self.wall_start[1]) / (time.monotonic() - self.wall_start[0])
      self.metrics.write(self.result_file, extra)
      print(f"benchmark result: {self.result_file} ({self.end_reason or 'ended'})", flush=True)

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
      self.blinker = blinker_for_command(m[1])
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

  def script_end_reason(self):
    """The overtake run ends a few seconds after the scripted driver is done."""
    if self.overtake is None or not self.overtake.finished_playing(self.sim_time() - OVERTAKE_TAIL_S):
      return None
    return "overtake"

  def _run(self, q):
    # imported here so the bridge module can be loaded without the sim deps
    from tools.sim.lib.simulated_car import SimulatedCar
    from tools.sim.lib.simulated_sensors import SimulatedSensors

    self.world = self.spawn_world(q)
    self.simulated_car = SimulatedCar()
    self.simulated_sensors = SimulatedSensors()
    # The car thread has its own SubMaster; never share one across threads.
    self.sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'lateralPlan', 'radarState',
                                  'longitudinalPlan', 'modelV2'])

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

      manual_control_active = time.monotonic() < self.manual_control_deadline
      if not manual_control_active:
        self.manual_throttle = self.manual_steer = self.manual_brake = 0.0

      if self.overtake is not None:
        armed = self.metrics is not None and self.metrics.moved_fast
        for cmd in self.overtake.commands(self.sim_time(), s.speed, armed):
          self._handle_command(cmd, s)

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
        action = next_cruise_action(cs.vCruise, self.cruise_kph_pending)
        if action is None:
          self.cruise_kph_pending = None
        else:
          self.held_button, self.held_frames = action
          self.cruise_press_wait = action[1] + CRUISE_PRESS_GAP

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

      if self.lead_log or self.metrics is not None:
        self.drain_collisions()
      if self.recorder is not None and self.rk.frame % self.TICKS_PER_FRAME == 0:
        self.record_frame()
      if (self.lead_log or self.metrics is not None) and self.rk.frame % 10 == 0:
        truth = self.world.truth_lead()
        if self.metrics is not None:
          self.score_sample(truth)
          self.end_reason = self.metrics.done_reason() or self.script_end_reason()
          if self.end_reason:
            self.shutdown()
        if self.lead_log:
          # once a second, and ten times a second while something is close ahead
          if self.rk.frame % 100 == 0 or (truth is not None and truth[0] < CLOSE_LEAD_LOG_M):
            self.print_status(truth)
          if truth is not None:
            self.save_frame_near_lead(truth)
      if not self.lead_log and self.rk.frame % 100 == 0:
        self.print_status()

      self.started.value = True
      self.rk.keep_time()

    self.finish_run()
