"""Per-run scoring for the obstacle benchmark. No CARLA or openpilot imports, so it is unit-testable offline.

Everything here is for measurement only: the simulator's ground truth is graded against what openpilot did and
is never given to openpilot (the real car has a camera and nothing else).
"""
import json
import math
import pathlib
from collections import deque

# kind -> (CARLA blueprint, yaw offset in degrees relative to the lane direction)
OBSTACLE_KINDS = {
  "motorcycle": ("vehicle.kawasaki.ninja", 0.0),
  "car": ("vehicle.audi.a2", 0.0),           # parked, seen from behind
  "car-across": ("vehicle.audi.a2", 90.0),   # lying across the lane
  "barrier": ("static.prop.streetbarrier", 90.0),
  "container": ("static.prop.container", 0.0),
}

# a contact with one of these is a collision with an obstacle; anything else (curb, pole, wall) is logged apart
OBSTACLE_TYPE_PREFIXES = ("vehicle.", "static.prop.")

SAMPLE_HZ = 10
DECEL_WINDOW_S = 0.5            # acceleration is the speed change over this window (smooth, independent of IMU noise)
HARD_BRAKE_MPS2 = -1.5          # deceleration threshold of the false braking count and of the "time below" metric
CLEAR_ROAD_M = 60.0             # no object within this distance counts as a clear road
CRUISE_REACHED = 0.9            # fraction of the set speed after which the cruise average is taken
STOPPED_MPS = 0.2
STOPPED_FOR_S = 5.0
AFTER_COLLISION_S = 2.0
STEADY_WINDOW_S = 10.0          # the steady-following figures are taken over the last this-many seconds


def parse_obstacle_spec(spec):
  """'kind:distance_m[:speed_kph],...' -> [(kind, distance_m, speed_kph)]. Speed defaults to 0 (world-fixed)."""
  out = []
  for item in spec.split(","):
    parts = item.strip().split(":")
    if len(parts) not in (2, 3):
      raise ValueError(f"obstacle '{item}' must be kind:distance_m[:speed_kph]")
    kind, distance = parts[0], float(parts[1])
    speed = float(parts[2]) if len(parts) == 3 else 0.0
    if kind not in OBSTACLE_KINDS:
      raise ValueError(f"obstacle kind '{kind}' must be one of: {', '.join(OBSTACLE_KINDS)}")
    if distance <= 0.0:
      raise ValueError(f"obstacle distance {distance:g} m must be positive")
    if speed < 0.0 or (speed > 0.0 and kind != "motorcycle"):
      raise ValueError("only a motorcycle can move (kind:distance_m:speed_kph with speed > 0)")
    out.append((kind, distance, speed))
  return out


def project_half_length(half_x, half_y, relative_yaw_deg):
  """Half-length of a box along the ego heading, given the box's yaw relative to the ego."""
  a = math.radians(relative_yaw_deg)
  return abs(math.cos(a)) * half_x + abs(math.sin(a)) * half_y


class OvertakeScript:
  """A scripted driver for the overtake scenario: the same commands the keyboard sends.

  Waits until the car has stood still for `still_s`, then plays `segments`, a list of (seconds, [commands]) where
  every command is re-sent every `repeat_s` as a held key is. `commands(now, speed)` returns what to send now.
  """
  DEFAULT_SEGMENTS = (
    (1.0, ("blinker_left",)),
    (2.0, ("steer_-0.15", "throttle_1.0")),   # turn out (a) with throttle (w)
    (2.0, ("steer_0.15", "throttle_1.0")),    # straighten (d)
    (4.0, ("throttle_1.0",)),                 # pass
    (0.5, ("blinker_off",)),
  )

  def __init__(self, segments=DEFAULT_SEGMENTS, still_s=3.0, repeat_s=0.1):
    self.segments = list(segments)
    self.still_s = still_s
    self.repeat_s = repeat_s
    self.still_since = None
    self.start = None
    self.last_sent = float("-inf")
    self.segment_index = -1
    self.now = 0.0

  @property
  def active(self):
    """True while the segments are playing (the scored part of the overtake)."""
    return self.start is not None and not self.finished_playing(self.now)

  @property
  def total_s(self):
    return sum(seconds for seconds, _ in self.segments)

  def finished_playing(self, now):
    return self.start is not None and now - self.start >= self.total_s

  def commands(self, now, speed, armed=True):
    self.now = now
    if self.start is None:
      if not armed or speed > STOPPED_MPS:
        self.still_since = None
        return []
      if self.still_since is None:
        self.still_since = now
      if now - self.still_since < self.still_s:
        return []
      self.start = now
    elapsed = now - self.start
    for index, (seconds, cmds) in enumerate(self.segments):
      if elapsed < seconds:
        new_segment = index != self.segment_index
        self.segment_index = index
        if new_segment or now - self.last_sent >= self.repeat_s:
          self.last_sent = now
          return list(cmds)
        return []
      elapsed -= seconds
    return []


class RunMetrics:
  """Collects one run's samples (10 Hz) and collisions, and summarises them as one JSON-able dict.

  A sample is (t, ego speed, truth, seen, set speed): truth is (bumper gap m, obstacle speed m/s) or None,
  seen is (dRel, vLead, prob) of the lead openpilot perceives or None. The clock starts at the first engaged
  sample, so engaging and the first acceleration are not scored as part of the scenario.
  """

  def __init__(self, scenario="", max_distance_m=None, stop_when_stopped=True):
    self.scenario = scenario
    self.max_distance_m = max_distance_m
    self.stop_when_stopped = stop_when_stopped
    self.t0 = None
    self.t = 0.0
    self.speeds = deque()          # (t, speed) for the acceleration window
    self.samples = 0
    self.distance_m = 0.0
    self.last_t = None
    self.min_gap = None
    self.first_lead_gap = None
    self.peak_decel = 0.0
    self.time_below = 0.0
    self.in_false_brake = False
    self.false_brake_events = 0
    self.set_speed_kph = None
    self.min_set_speed_kph = None
    self.cruise_distance = 0.0
    self.cruise_time = 0.0
    self.reached_cruise = False
    self.final_speed = 0.0
    self.moved_fast = False
    self.stopped_since = None
    self.collision = None          # (other, impact speed, t)
    self.other_contacts = 0
    self.other_contact_types = set()
    self.brake_against_input = 0.0
    self.yrel_errors = []
    self.end_reason = None
    self.approach_speed = None     # ego speed when the obstacle first came within CLEAR_ROAD_M
    self.steady = deque()          # (t, gap, ego speed, obstacle speed) over the last STEADY_WINDOW_S

  def sample(self, now, speed, truth, seen, set_speed_kph, brake=0.0, engaged=True, in_overtake=False,
             y_rel_pair=None):
    if self.t0 is None:
      if not engaged:
        return
      self.t0 = now
    self.t = now - self.t0
    self.samples += 1
    if self.last_t is not None:
      dt = self.t - self.last_t
      self.distance_m += speed * dt
      if self.reached_cruise:
        self.cruise_distance += speed * dt
        self.cruise_time += dt
    self.last_t = self.t
    self.final_speed = speed
    self.set_speed_kph = set_speed_kph
    self.min_set_speed_kph = set_speed_kph if self.min_set_speed_kph is None else min(self.min_set_speed_kph, set_speed_kph)
    if not self.reached_cruise and set_speed_kph and speed >= CRUISE_REACHED * set_speed_kph / 3.6:
      self.reached_cruise = True

    if truth is not None:
      gap = truth[0]
      self.min_gap = gap if self.min_gap is None else min(self.min_gap, gap)
      if seen is not None and self.first_lead_gap is None:
        self.first_lead_gap = gap
      self.steady.append((self.t, gap, speed, truth[1]))
      if self.approach_speed is None and gap < CLEAR_ROAD_M:
        self.approach_speed = speed
    while self.steady and self.t - self.steady[0][0] > STEADY_WINDOW_S:
      self.steady.popleft()

    self.speeds.append((self.t, speed))
    while len(self.speeds) > 1 and self.t - self.speeds[1][0] >= DECEL_WINDOW_S:
      self.speeds.popleft()
    span = self.t - self.speeds[0][0]
    # after any contact the speed change is the impact itself, not braking: the braking figures stop there
    if span >= DECEL_WINDOW_S * 0.9 and self.collision is None and self.other_contacts == 0:
      accel = (speed - self.speeds[0][1]) / span
      self.peak_decel = min(self.peak_decel, accel)
      if accel < HARD_BRAKE_MPS2:
        self.time_below += 1.0 / SAMPLE_HZ
      clear = truth is None or truth[0] > CLEAR_ROAD_M
      if accel < HARD_BRAKE_MPS2 and clear:
        if not self.in_false_brake:
          self.false_brake_events += 1
        self.in_false_brake = True
      elif accel >= HARD_BRAKE_MPS2:
        self.in_false_brake = False

    if speed > 2.0:
      self.moved_fast = True
    if speed < STOPPED_MPS:
      self.stopped_since = self.t if self.stopped_since is None else self.stopped_since
    else:
      self.stopped_since = None

    if in_overtake:
      self.brake_against_input = max(self.brake_against_input, brake)
      if y_rel_pair is not None:
        self.yrel_errors.append(y_rel_pair[0] - y_rel_pair[1])

  def collide(self, other, speed):
    if other.startswith(OBSTACLE_TYPE_PREFIXES):
      if self.collision is None:
        self.collision = (other, speed, self.t)
    else:
      self.other_contacts += 1
      self.other_contact_types.add(other)

  def done_reason(self):
    """Why the run can end now, or None. Collisions end the run shortly after contact."""
    if self.collision is not None and self.t - self.collision[2] >= AFTER_COLLISION_S:
      return "collision"
    if self.max_distance_m is not None and self.distance_m >= self.max_distance_m:
      return "distance"
    if (self.stop_when_stopped and self.moved_fast and self.stopped_since is not None
        and self.t - self.stopped_since >= STOPPED_FOR_S):
      return "stopped"
    return None

  def summary(self):
    collided = self.collision is not None
    cruise_avg = self.cruise_distance / self.cruise_time if self.cruise_time > 0.0 else None
    errors = self.yrel_errors
    window = list(self.steady)
    steady = {}
    if window and window[-1][0] == self.t and window[0][0] <= self.t - STEADY_WINDOW_S + 1.0:
      gaps = [w[1] for w in window]
      steady = {"steady_gap_m": round(sum(gaps) / len(gaps), 2), "steady_gap_range_m": round(max(gaps) - min(gaps), 2),
                "steady_ego_speed_mps": round(sum(w[2] for w in window) / len(window), 2),
                "steady_obstacle_speed_mps": round(sum(w[3] for w in window) / len(window), 2)}
    return {
      **steady,
      "scenario": self.scenario,
      "engaged": self.t0 is not None,
      "end_reason": self.end_reason,
      "duration_s": round(self.t, 2),
      "distance_m": round(self.distance_m, 1),
      "min_gap_m": None if self.min_gap is None else round(self.min_gap, 2),
      "collision": collided,
      "collided_with": self.collision[0] if collided else None,
      "impact_speed_mps": round(self.collision[1], 2) if collided else None,
      "other_contacts": self.other_contacts,
      "other_contact_types": sorted(self.other_contact_types),
      "peak_decel_mps2": round(self.peak_decel, 2),
      "time_below_m1_5_s": round(self.time_below, 1),
      "false_brake_events": self.false_brake_events,
      "final_speed_mps": round(self.final_speed, 2),
      "avg_speed_cruise_mps": None if cruise_avg is None else round(cruise_avg, 2),
      "set_speed_final_kph": self.set_speed_kph,
      "set_speed_min_kph": self.min_set_speed_kph,
      "approach_speed_mps": None if self.approach_speed is None else round(self.approach_speed, 2),
      "first_lead_true_gap_m": None if self.first_lead_gap is None else round(self.first_lead_gap, 1),
      "overtake_max_brake": round(self.brake_against_input, 2),
      "overtake_yrel_samples": len(errors),
      "overtake_yrel_mean_abs_error_m": round(sum(abs(e) for e in errors) / len(errors), 2) if errors else None,
    }

  def write(self, path, extra=None):
    """Write the summary as JSON; `extra` adds figures only the bridge knows (None values are left out)."""
    summary = self.summary()
    summary.update({k: round(float(v), 2) for k, v in (extra or {}).items() if v is not None})
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2) + "\n")


class FrameRecorder:
  """Writes every Nth camera frame (JPEG) and one ground-truth line per frame to <dir>/truth.jsonl.

  The offline trials of the clear-distance source read these back. Encoding runs on a small thread pool.
  """

  def __init__(self, directory, every=1, quality=90, images=True):
    from concurrent.futures import ThreadPoolExecutor
    self.directory = pathlib.Path(directory)
    self.directory.mkdir(parents=True, exist_ok=True)
    self.every = max(int(every), 1)
    self.quality = quality
    self.images = images
    self.count = 0
    self.saved = 0
    self.pool = ThreadPoolExecutor(max_workers=2)
    self.truth_file = open(self.directory / "truth.jsonl", "a")

  def add(self, bgra, record):
    """`bgra` is copied here; `record` is a JSON-able dict for this frame."""
    index = self.count
    self.count += 1
    if index % self.every:
      return
    name = f"{self.saved:06d}.jpg"
    self.saved += 1
    self.truth_file.write(json.dumps({"frame": name, **record}) + "\n")
    self.truth_file.flush()
    if self.images:
      self.pool.submit(self._save, bgra.copy(), self.directory / name)

  def _save(self, bgra, path):
    from PIL import Image
    Image.fromarray(bgra[:, :, [2, 1, 0]]).save(path, quality=self.quality)

  def close(self):
    self.pool.shutdown(wait=True)
    self.truth_file.close()
