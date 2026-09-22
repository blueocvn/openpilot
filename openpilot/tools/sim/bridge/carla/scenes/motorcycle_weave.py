"""Deterministic schedule and CARLA runtime for motorcycle cut-ins."""

from collections import deque
from dataclasses import dataclass, replace
import hashlib
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import threading
import time

from openpilot.tools.sim.bridge.carla.scenes.report import RotatingReport


DEFAULT_EGO_SPEED_MPS = 30.0 / 3.6
DEFAULT_MOTORCYCLE_SPEED_MPS = 6.0
DEFAULT_CLEARANCE_M = 12.0
DEFAULT_DURATION_S = 4.0
DEFAULT_FIRST_START_S = 6.0
DEFAULT_INTERVAL_S = 4.0
DEFAULT_EVENT_COUNT = 6
SCENE_READY_SPEED_MPS = 7.5
STAGING_GAP_MARGIN_M = 1.2


@dataclass(frozen=True)
class WeaveEvent:
  direction: str
  start_time_s: float
  duration_s: float
  initial_distance_m: float


@dataclass(frozen=True)
class ContinuousPass:
  direction: str
  start_time_s: float
  interval_s: float
  speed_mps: float
  merge_gap_m: float
  entry_s: float
  hold_s: float
  exit_s: float
  spawn_center_distance_m: float

  @property
  def total_s(self) -> float:
    return self.entry_s + self.hold_s + self.exit_s

  @property
  def next_start_time_s(self) -> float:
    return self.start_time_s + self.interval_s


def pass_lateral_progress(event: ContinuousPass, elapsed_s: float) -> float:
  if elapsed_s < event.entry_s:
    return 0.5 * quintic_progress(elapsed_s / event.entry_s)
  if elapsed_s < event.entry_s + event.hold_s:
    return 0.5
  return 0.5 + 0.5 * quintic_progress((elapsed_s - event.entry_s - event.hold_s) / event.exit_s)


class ContinuousWeaveScheduler:
  """Seeded event timing; scene owns CARLA actors and validates their placement."""

  def __init__(self, case: str, seed: int):
    directions = {
      "right_to_left": ("right_to_left",),
      "left_to_right": ("left_to_right",),
      "alternating": ("right_to_left", "left_to_right"),
    }
    if case not in directions:
      raise ValueError(f"unknown motorcycle weave case: {case}")
    self.directions = directions[case]
    self.rng = random.Random(seed)
    self.next_due_s = 0.0
    self.event_index = 0
    self._pending = None
    self._pending_rng_state = None

  def propose(self, now_s: float, *, ego_speed_mps: float, available_road_m: float,
              actor_available: bool, vehicle_length_buffer_m: float = 3.5) -> ContinuousPass | None:
    if now_s < self.next_due_s or ego_speed_mps < SCENE_READY_SPEED_MPS or not actor_available:
      return None
    state = self.rng.getstate()
    interval_s = self.rng.uniform(2.0, 3.0)
    speed_mps = self.rng.uniform(4.5, 5.5)
    merge_gap_m = self.rng.uniform(6.0, 8.0)
    entry_s = self.rng.uniform(1.15, 1.55)
    # The physical Vespa reaches and leaves the lane center after the
    # reference changes phase; this shorter programmed plateau targets a
    # measured 1.5–2.0 s lane occupation.
    hold_s = self.rng.uniform(0.75, 1.05)
    exit_s = self.rng.uniform(1.3, 1.6)
    start_distance = merge_gap_m + vehicle_length_buffer_m + (ego_speed_mps - speed_mps) * entry_s
    event = ContinuousPass(self.directions[self.event_index % len(self.directions)], now_s, interval_s,
                           speed_mps, merge_gap_m, entry_s, hold_s, exit_s, start_distance)
    next_rng_state = self.rng.getstate()
    self.rng.setstate(state)
    if available_road_m < start_distance + speed_mps * event.total_s + 20.0:
      return None
    self._pending = event
    self._pending_rng_state = next_rng_state
    return event

  def commit(self, event: ContinuousPass):
    if event != self._pending:
      raise ValueError("event is not the current proposal")
    self.rng.setstate(self._pending_rng_state)
    self.event_index += 1
    self.next_due_s = event.next_start_time_s
    self._pending = None
    self._pending_rng_state = None


@dataclass(frozen=True)
class BackgroundSlot:
  side: str
  distance_m: float
  speed_mps: float


class BackgroundTrafficScheduler:
  def __init__(self, seed):
    self.rng = random.Random(seed ^ 0xD31E)
    self.target_count = self.rng.randint(3, 5)
    self.index = 0

  def next_slot(self):
    state = self.rng.getstate()
    slot = BackgroundSlot("left" if self.index % 2 == 0 else "right",
                          25.0 + 5.0 * (self.index % 4) + self.rng.uniform(0.0, 3.0),
                          self.rng.uniform(5.0, 7.0))
    self.rng.setstate(state)
    return slot

  def commit(self):
    self.rng.uniform(0.0, 3.0)
    self.rng.uniform(5.0, 7.0)
    self.index += 1


def quintic_progress(progress: float) -> float:
  """Return a zero-velocity, zero-acceleration lane-change progress value."""
  u = min(max(float(progress), 0.0), 1.0)
  return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


def event_progress(event: WeaveEvent, elapsed_s: float) -> float:
  """Return a smooth [0, 1] lane-change progress for an event window."""
  return quintic_progress((elapsed_s - event.start_time_s) / event.duration_s)


def lookahead_event_progress(event: WeaveEvent, elapsed_s: float, *, lookahead_m: float, speed_mps: float) -> float:
  """Return lateral progress at the time the actor reaches its lookahead point."""
  if speed_mps <= 0.0:
    raise ValueError("speed_mps must be positive")
  return event_progress(event, elapsed_s + lookahead_m / speed_mps)


def same_lane_path(first, second) -> bool:
  """Whether two CARLA waypoints continue along the same road lane."""
  return first.road_id == second.road_id and first.lane_id == second.lane_id


def finite_or_none(value):
  value = float(value)
  return value if math.isfinite(value) else None


def sha256_path(path):
  with Path(path).open("rb") as source:
    return hashlib.file_digest(source, "sha256").hexdigest()


def param_value_text(value):
  return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else str(value)


def radar_lead_snapshot(lead):
  return {
    "present": bool(lead.present),
    "distance_m": finite_or_none(lead.dRel),
    "lateral_m": finite_or_none(lead.yRel),
    "relative_speed_mps": finite_or_none(lead.vRel),
    "lead_speed_mps": finite_or_none(lead.vLead),
    "model_probability": finite_or_none(lead.modelProb),
    "radar_matched": bool(lead.radar),
  }


def find_middle_driving_lane(start, driving_lane_type):
  """Find the nearest same-direction lane with driving lanes on both sides."""
  queue = [start]
  seen = set()

  def valid(candidate):
    return (candidate is not None and candidate.lane_type == driving_lane_type and
            candidate.road_id == start.road_id and candidate.lane_id * start.lane_id > 0)

  while queue:
    candidate = queue.pop(0)
    if id(candidate) in seen or not valid(candidate):
      continue
    seen.add(id(candidate))
    left = candidate.get_left_lane()
    right = candidate.get_right_lane()
    if valid(left) and valid(right):
      return candidate
    queue.extend((left, right))
  raise RuntimeError("motorcycle_weave needs three same-direction driving lanes")


def relocated_spawn_height(spawn_height, source_lane_height, target_lane_height):
  """Preserve the spawn point's ground clearance after moving across lanes."""
  return target_lane_height + (spawn_height - source_lane_height)


def scenario_ready(*, active, long_active, ego_speed_mps):
  return bool(active and long_active) and ego_speed_mps >= SCENE_READY_SPEED_MPS


def trajectory_distance(initial_distance_m, speed_mps, elapsed_s, *, lookahead_m):
  return initial_distance_m + speed_mps * elapsed_s + lookahead_m


def advance_path_distance(distance_m, *, velocity_x, velocity_y, route_yaw_deg, dt_s):
  """Advance the reference by measured forward travel, never by scene clock."""
  yaw = math.radians(route_yaw_deg)
  forward_speed = math.cos(yaw) * velocity_x + math.sin(yaw) * velocity_y
  return distance_m + max(forward_speed, 0.0) * max(dt_s, 0.0)


def background_cut_in_ready(event, *, distance_m, ego_speed_mps, bike_speed_mps, vehicle_length_buffer_m):
  if abs(bike_speed_mps - event.speed_mps) > 1.0:
    return False
  required_distance = (event.merge_gap_m + STAGING_GAP_MARGIN_M + vehicle_length_buffer_m +
                       (ego_speed_mps - bike_speed_mps) * event.entry_s)
  return abs(distance_m - required_distance) <= 0.5


def ego_relative_longitudinal(ego_x: float, ego_y: float, actor_x: float, actor_y: float,
                              ego_yaw_deg: float) -> float:
  yaw = math.radians(ego_yaw_deg)
  return math.cos(yaw) * (actor_x - ego_x) + math.sin(yaw) * (actor_y - ego_y)


def should_retire_completed(*, pass_elapsed_s: float, total_s: float, lateral_error_m: float) -> bool:
  return (pass_elapsed_s >= total_s + 0.5 and abs(lateral_error_m) <= 1.2 or
          pass_elapsed_s >= total_s + 2.0)


def tracking_error_components(*, actual_x, actual_y, reference_x, reference_y, reference_yaw_deg):
  """Return signed longitudinal and lateral error in the reference lane frame."""
  yaw = math.radians(reference_yaw_deg)
  dx, dy = actual_x - reference_x, actual_y - reference_y
  return (
    math.cos(yaw) * dx + math.sin(yaw) * dy,
    -math.sin(yaw) * dx + math.cos(yaw) * dy,
  )


def motorcycle_longitudinal_control(speed_mps, integral, target_speed_mps=DEFAULT_MOTORCYCLE_SPEED_MPS):
  speed_error = target_speed_mps - speed_mps
  if speed_error < -0.25:
    return 0.0, max(0.0, min(0.4, -0.2 * speed_error)), 0.0
  integral = max(-0.2, min(0.5, integral + 0.08 * speed_error * 0.05))
  return max(0.0, min(0.9, 0.45 + 0.35 * speed_error + integral)), 0.0, integral


def motorcycle_steer_command(*, local_x, local_y):
  return max(-1.0, min(1.0, 1.6 * math.atan2(local_y, max(local_x, 0.5))))


def _convex_hull(points):
  points = sorted(set(points))
  if len(points) <= 1:
    return points

  def cross(origin, first, second):
    return ((first[0] - origin[0]) * (second[1] - origin[1]) -
            (first[1] - origin[1]) * (second[0] - origin[0]))

  lower = []
  for point in points:
    while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
      lower.pop()
    lower.append(point)
  upper = []
  for point in reversed(points):
    while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
      upper.pop()
    upper.append(point)
  return lower[:-1] + upper[:-1]


def _point_segment_distance(point, start, end):
  dx, dy = end[0] - start[0], end[1] - start[1]
  length_squared = dx * dx + dy * dy
  if length_squared == 0.0:
    return math.hypot(point[0] - start[0], point[1] - start[1])
  projection = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
  nearest = (start[0] + projection * dx, start[1] + projection * dy)
  return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def _inside_convex(point, polygon):
  signs = []
  for index, start in enumerate(polygon):
    end = polygon[(index + 1) % len(polygon)]
    cross = (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (point[0] - start[0])
    if abs(cross) > 1e-9:
      signs.append(cross > 0.0)
  return not signs or all(sign == signs[0] for sign in signs)


def polygon_clearance(first, second):
  """Return minimum 2D distance between two convex vehicle footprints."""
  first = _convex_hull(first)
  second = _convex_hull(second)
  if not first or not second:
    raise ValueError("vehicle footprint cannot be empty")
  if _inside_convex(first[0], second) or _inside_convex(second[0], first):
    return 0.0
  distances = []
  for points, edges in ((first, second), (second, first)):
    for point in points:
      for index, start in enumerate(edges):
        distances.append(_point_segment_distance(point, start, edges[(index + 1) % len(edges)]))
  return min(distances)


def proposed_motorcycle_footprint(x, y, yaw_deg):
  yaw = math.radians(yaw_deg)
  forward = (math.cos(yaw), math.sin(yaw))
  right = (-forward[1], forward[0])
  return [(x + forward[0] * along + right[0] * across,
           y + forward[1] * along + right[1] * across)
          for along, across in ((-1.1, -0.5), (1.1, -0.5), (1.1, 0.5), (-1.1, 0.5))]


def spawn_footprints_clear(candidate, ego, actors, *, ego_margin_m=4.0, actor_margin_m=6.0):
  return (polygon_clearance(candidate, ego) >= ego_margin_m and
          all(polygon_clearance(candidate, actor) >= actor_margin_m for actor in actors))


def build_weave_events(case: str, *, ego_speed_mps: float = DEFAULT_EGO_SPEED_MPS,
                       motorcycle_speed_mps: float = DEFAULT_MOTORCYCLE_SPEED_MPS,
                       clearance_m: float = DEFAULT_CLEARANCE_M,
                       duration_s: float = DEFAULT_DURATION_S,
                       first_start_s: float = DEFAULT_FIRST_START_S,
                       interval_s: float = DEFAULT_INTERVAL_S,
                       event_count: int = DEFAULT_EVENT_COUNT) -> list[WeaveEvent]:
  """Build the fixed Phase 1 event schedule without using wall-clock time.

  Each actor is initially placed far enough ahead that it passes the ego
  vehicle at ``clearance_m`` midway through its lane change.  This makes the
  scenario reproducible despite actors beginning their manoeuvres at different
  simulation times.
  """
  directions = {
    "right_to_left": ("right_to_left",),
    "left_to_right": ("left_to_right",),
    "alternating": ("right_to_left", "left_to_right"),
  }
  if case not in directions:
    raise ValueError(f"unknown motorcycle weave case: {case}")
  if event_count < 1 or duration_s <= 0.0 or interval_s <= 0.0:
    raise ValueError("event_count, duration_s, and interval_s must be positive")

  events = []
  relative_speed = ego_speed_mps - motorcycle_speed_mps
  for index in range(event_count):
    start_time_s = first_start_s + index * interval_s
    initial_distance_m = clearance_m + relative_speed * (start_time_s + duration_s / 2.0)
    events.append(WeaveEvent(
      direction=directions[case][index % len(directions[case])],
      start_time_s=start_time_s,
      duration_s=duration_s,
      initial_distance_m=initial_distance_m,
    ))
  return events


class MotorcycleWeaveScene:
  """Own CARLA motorcycle actors while leaving world ticking to ``CarlaWorld``.

  The bridge calls ``before_tick`` once per synchronous CARLA tick.  The scene
  only submits actor controls; it never calls ``world.tick()``, preserving the
  single CARLA clock owner required for repeatable timing.
  """

  LOOKAHEAD_M = 3.0
  EXIT_LATERAL_LOOKAHEAD_M = 1.2

  def __init__(self, carla, world, ego_vehicle, actor_sink, *, case="alternating", seed=42,
               duration_s=45.0, report_dir=None):
    if duration_s < 0:
      raise ValueError("duration_s must be nonnegative")
    self.carla = carla
    self.world = world
    self.ego_vehicle = ego_vehicle
    self.actor_sink = actor_sink
    self.case = case
    self.seed = seed
    self.duration_s = duration_s
    self.scheduler = ContinuousWeaveScheduler(case, seed)
    self.background_scheduler = BackgroundTrafficScheduler(seed)
    self.ready_time_s = None
    self.start_time_s = None
    self.finished = False
    self._actors = []
    self._background = []
    self._collisions = deque(maxlen=32)
    self._last_sample_time_s = None
    self._last_control_sample_s = None
    self.latest_openpilot = {"available": False}
    self.latest_bridge_control = {"controller_active": False}
    root = Path(report_dir) if report_dir else Path(".carla/reports")
    self.report_dir = root / f"motorcycle-weave-{time.strftime('%Y%m%d-%H%M%S')}-{time.time_ns() % 1_000_000_000:09d}"
    self.report_file = None
    self._manifest_thread = None

  def start_manifest(self):
    self._manifest_thread = threading.Thread(target=self._write_manifest, name="carla-scene-manifest")
    self._manifest_thread.start()

  def set_openpilot_ready(self, simulation_time_s):
    if self.ready_time_s is None:
      self.ready_time_s = simulation_time_s

  def before_tick(self, simulation_time_s):
    if self.ready_time_s is None or self.finished:
      return
    if self.start_time_s is None:
      self.start_time_s = simulation_time_s
      self._prepare()

    elapsed_s = simulation_time_s - self.start_time_s
    if self.duration_s > 0 and elapsed_s >= self.duration_s:
      self.finished = True
      self._write({"type": "scenario_finished", "scene_time_s": elapsed_s})
      return

    for item in self._actors[:]:
      if not item["actor"].is_alive:
        continue
      event = item["event"]
      pass_elapsed_s = elapsed_s - event.start_time_s
      self._advance_actor_progress(item, simulation_time_s)
      self._track_lane_occupancy(item, elapsed_s)
      control_progress = self._control_progress(item)
      try:
        target = self._target_location(item, pass_elapsed_s, control_progress, lookahead_m=self.LOOKAHEAD_M)
      except RuntimeError:
        item["actor"].apply_control(self.carla.VehicleControl(throttle=0.0, brake=1.0))
        continue
      self._apply_control(item, target)
      if (item["progress_m"] >= event.speed_mps * event.total_s and
          item["left_ego_lane_at"] is not None and not item["completed"]):
        item["completed"] = True
        self._write({"type": "cut_in_finished", "scene_time_s": elapsed_s,
                     "actor_id": item["actor"].id, "direction": event.direction})
      if item["completed"] and pass_elapsed_s >= event.total_s + 0.5:
        try:
          target, yaw = self._target_pose(item, pass_elapsed_s, 1.0, lookahead_m=0.0)
          actor_location = item["actor"].get_location()
          _, lateral_error = tracking_error_components(
            actual_x=actor_location.x, actual_y=actor_location.y,
            reference_x=target.x, reference_y=target.y, reference_yaw_deg=yaw)
        except RuntimeError:
          lateral_error = math.inf
        if should_retire_completed(pass_elapsed_s=pass_elapsed_s, total_s=event.total_s,
                                   lateral_error_m=lateral_error):
          self._retire_actor(item, elapsed_s)

    self._update_background(elapsed_s)

    if not self.latest_openpilot.get("active", False) or not self.latest_openpilot.get("long_active", False):
      return
    lanes = self._candidate_lanes()
    if lanes is None:
      return
    ego_lane, left_lane, right_lane, available_road_m = lanes
    if self._ego_speed() >= SCENE_READY_SPEED_MPS:
      self._maintain_background(elapsed_s, left_lane, right_lane)
    if elapsed_s < self.scheduler.next_due_s:
      return
    direction = self.scheduler.directions[self.scheduler.event_index % len(self.scheduler.directions)]
    if any(not item["completed"] for item in self._actors):
      return
    side = "right" if direction == "right_to_left" else "left"
    candidates = [item for item in self._background if item["side"] == side and item["actor"].is_alive]
    if not candidates:
      return
    event = self.scheduler.propose(elapsed_s, ego_speed_mps=self._ego_speed(), available_road_m=available_road_m,
                                   actor_available=True, vehicle_length_buffer_m=self._vehicle_length_buffer(candidates[0]))
    if event is None:
      return
    ego_transform = self.ego_vehicle.get_transform()
    candidates.sort(key=lambda item: ego_relative_longitudinal(
      ego_transform.location.x, ego_transform.location.y,
      item["actor"].get_location().x, item["actor"].get_location().y,
      ego_transform.rotation.yaw))
    for item in candidates:
      actor = item["actor"]
      actor_position = actor.get_location()
      center_distance = ego_relative_longitudinal(
        ego_transform.location.x, ego_transform.location.y,
        actor_position.x, actor_position.y, ego_transform.rotation.yaw)
      buffer_m = self._vehicle_length_buffer(item)
      if center_distance < event.merge_gap_m + buffer_m:
        continue
      item["speed_mps"] = event.speed_mps
      item["staged_for_cut_in"] = True
      velocity = actor.get_velocity()
      actor_speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
      if background_cut_in_ready(event, distance_m=center_distance, ego_speed_mps=self._ego_speed(),
                                 bike_speed_mps=actor_speed, vehicle_length_buffer_m=buffer_m):
        if self._start_pass(event, ego_lane, left_lane, right_lane, item):
          self.scheduler.commit(event)
        return

  def _update_background(self, elapsed_s):
    if not self._background:
      return
    ego = self.ego_vehicle.get_transform()
    for item in self._background[:]:
      actor = item["actor"]
      if not actor.is_alive:
        self._background.remove(item)
        if actor in self.actor_sink:
          self.actor_sink.remove(actor)
        continue
      transform = actor.get_transform()
      relative = ego_relative_longitudinal(
        ego.location.x, ego.location.y, transform.location.x, transform.location.y, ego.rotation.yaw)
      if relative < -12.0 or relative > 80.0:
        self._retire_background(item, elapsed_s)
        continue
      try:
        current_lane = self.world.get_map().get_waypoint(
          transform.location, project_to_road=True, lane_type=self.carla.LaneType.Driving)
        if current_lane is None:
          raise RuntimeError("background motorcycle left the driving lane")
        target = self._advance(current_lane, self.LOOKAHEAD_M).transform.location
      except RuntimeError:
        self._retire_background(item, elapsed_s)
        continue
      self._apply_control(item, target)

  def _retire_background(self, item, elapsed_s):
    actor = item["actor"]
    actor.destroy()
    self._background.remove(item)
    self.actor_sink.remove(actor)
    self._write({"type": "background_retired", "scene_time_s": elapsed_s, "actor_id": actor.id})

  def _maintain_background(self, elapsed_s, left_lane, right_lane):
    if len(self._background) >= self.background_scheduler.target_count:
      return
    slot = self.background_scheduler.next_slot()
    lane = left_lane if slot.side == "left" else right_lane
    try:
      transform = self._advance(lane, slot.distance_m).transform
    except RuntimeError:
      return
    transform.location.z += 0.15
    if not self._spawn_clear(transform):
      return
    blueprint = self.world.get_blueprint_library().find("vehicle.vespa.zx125")
    actor = self.world.try_spawn_actor(blueprint, transform)
    if actor is None:
      return
    forward = transform.get_forward_vector()
    actor.set_target_velocity(self.carla.Vector3D(x=forward.x * slot.speed_mps,
                                                  y=forward.y * slot.speed_mps, z=0.0))
    self.actor_sink.append(actor)
    self._background.append({"actor": actor, "source": lane, "distance_m": slot.distance_m,
                             "start_s": elapsed_s, "side": slot.side, "speed_mps": slot.speed_mps,
                             "speed_integral": 0.0})
    self.background_scheduler.commit()
    self._write({"type": "background_started", "scene_time_s": elapsed_s, "actor_id": actor.id,
                 "side": slot.side, "speed_mps": slot.speed_mps})

  def _spawn_clear(self, transform):
    location = transform.location
    candidate = proposed_motorcycle_footprint(location.x, location.y, transform.rotation.yaw)
    ego_transform = self.ego_vehicle.get_transform()
    ego_polygon = [(vertex.x, vertex.y) for vertex in
                   self.ego_vehicle.bounding_box.get_world_vertices(ego_transform)]
    actor_polygons = [[(vertex.x, vertex.y) for vertex in item["actor"].bounding_box.get_world_vertices(item["actor"].get_transform())]
                      for item in (*self._actors, *self._background) if item["actor"].is_alive]
    return spawn_footprints_clear(candidate, ego_polygon, actor_polygons)

  def after_tick(self, simulation_time_s):
    if self.start_time_s is None:
      return
    elapsed_s = simulation_time_s - self.start_time_s
    if self._last_control_sample_s is None or elapsed_s - self._last_control_sample_s >= 0.1 - 1e-6:
      self._last_control_sample_s = elapsed_s
      ego_velocity = self.ego_vehicle.get_velocity()
      get_acceleration = getattr(self.ego_vehicle, "get_acceleration", None)
      ego_acceleration = get_acceleration() if get_acceleration is not None else None
      ego_transform = self.ego_vehicle.get_transform()
      get_forward_vector = getattr(ego_transform, "get_forward_vector", None)
      forward = get_forward_vector() if get_forward_vector is not None else None
      longitudinal_accel = (ego_acceleration.x * forward.x + ego_acceleration.y * forward.y
                            if ego_acceleration is not None and forward is not None else None)
      self._write({
        "type": "control_sample", "scene_time_s": elapsed_s,
        "ego_speed_mps": math.sqrt(ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2),
        "ego_accel_mps2": finite_or_none(longitudinal_accel) if longitudinal_accel is not None else None,
        "ego_brake": float(self.ego_vehicle.get_control().brake),
        "openpilot": self.latest_openpilot,
        "bridge_control": self.latest_bridge_control,
      })
    if self._last_sample_time_s is not None and elapsed_s - self._last_sample_time_s < 0.5 - 1e-6:
      return
    self._last_sample_time_s = elapsed_s
    ego_transform = self.ego_vehicle.get_transform()
    ego_velocity = self.ego_vehicle.get_velocity()
    ego_control = self.ego_vehicle.get_control()
    actors = []
    for item in self._actors:
      actor = item["actor"]
      if not actor.is_alive:
        continue
      transform = actor.get_transform()
      velocity = actor.get_velocity()
      event = item["event"]
      pass_elapsed_s = elapsed_s - event.start_time_s
      try:
        target, reference_yaw = self._target_pose(
          item, pass_elapsed_s, pass_lateral_progress(event, item["progress_m"] / event.speed_mps), lookahead_m=0.0,
        )
      except RuntimeError:
        continue
      longitudinal_error, lateral_error = tracking_error_components(
        actual_x=transform.location.x,
        actual_y=transform.location.y,
        reference_x=target.x,
        reference_y=target.y,
        reference_yaw_deg=reference_yaw,
      )
      ego_footprint = [(vertex.x, vertex.y) for vertex in self.ego_vehicle.bounding_box.get_world_vertices(ego_transform)]
      actor_footprint = [(vertex.x, vertex.y) for vertex in actor.bounding_box.get_world_vertices(transform)]
      relative_x = ego_relative_longitudinal(
        ego_transform.location.x, ego_transform.location.y,
        transform.location.x, transform.location.y, ego_transform.rotation.yaw)
      _, relative_y = tracking_error_components(
        actual_x=transform.location.x, actual_y=transform.location.y,
        reference_x=ego_transform.location.x, reference_y=ego_transform.location.y,
        reference_yaw_deg=ego_transform.rotation.yaw)
      actors.append({
        "id": actor.id,
        "role": "cut_in",
        "direction": item["event"].direction,
        "phase": pass_lateral_progress(event, item["progress_m"] / event.speed_mps),
        "actual_path_distance_m": item["progress_m"],
        "in_ego_lane": item["entered_ego_lane_at"] is not None and item["left_ego_lane_at"] is None,
        "actual_hold_s": (elapsed_s - item["entered_ego_lane_at"]
                          if item["entered_ego_lane_at"] is not None and item["left_ego_lane_at"] is None else
                          item["left_ego_lane_at"] - item["entered_ego_lane_at"]
                          if item["left_ego_lane_at"] is not None else None),
        "target_speed_mps": event.speed_mps,
        "position_m": [transform.location.x, transform.location.y, transform.location.z],
        "speed_mps": math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
        "tracking_error_m": math.hypot(transform.location.x - target.x, transform.location.y - target.y),
        "longitudinal_tracking_error_m": longitudinal_error,
        "lateral_tracking_error_m": lateral_error,
        "motorcycle_steer": item.get("last_steer"),
        "distance_to_ego_m": polygon_clearance(ego_footprint, actor_footprint),
        "ego_forward_gap_m": relative_x - self.ego_vehicle.bounding_box.extent.x - actor.bounding_box.extent.x,
        "ego_lateral_offset_m": relative_y,
      })
    for item in self._background:
      actor = item["actor"]
      if not actor.is_alive:
        continue
      transform = actor.get_transform()
      velocity = actor.get_velocity()
      actors.append({
        "id": actor.id, "role": "staged_cut_in" if item.get("staged_for_cut_in") else "background",
        "position_m": [transform.location.x, transform.location.y, transform.location.z],
        "speed_mps": math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
        "target_speed_mps": item["speed_mps"],
      })
    self._write({
      "type": "ground_truth",
      "scene_time_s": elapsed_s,
      "ego_speed_mps": math.sqrt(ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2),
      "ego_brake": float(ego_control.brake),
      "ego_throttle": float(ego_control.throttle),
      "actors": actors,
      "collisions": list(self._collisions),
      "openpilot": self.latest_openpilot,
      "bridge_control": self.latest_bridge_control,
    })

  def record_openpilot(self, sm):
    try:
      car_state = sm["carState"]
      car_control = sm["carControl"]
      car_output = sm["carOutput"]
      selfdrive_state = sm["selfdriveState"]
      plan = sm["longitudinalPlan"]
      model = sm["modelV2"]
      lead = model.leadsV3[0] if len(model.leadsV3) else None
      try:
        radar = sm["radarState"]
      except (KeyError, TypeError):
        radar = None
      try:
        onroad_events = [str(event.name) for event in sm["onroadEvents"]]
      except (KeyError, TypeError):
        onroad_events = []
      services = ("carState", "carControl", "carOutput", "selfdriveState", "longitudinalPlan", "modelV2", "radarState", "onroadEvents")
      valid = getattr(sm, "valid", {})
      alive = getattr(sm, "alive", {})
      mono_time = getattr(sm, "logMonoTime", {})
      self.latest_openpilot = {
        "available": True,
        "active": bool(selfdrive_state.active),
        "long_active": bool(car_control.longActive),
        "selfdrive_state": str(getattr(selfdrive_state, "state", "unknown")),
        "alert_type": str(getattr(selfdrive_state, "alertType", "")),
        "onroad_events": onroad_events,
        "cruise_enabled": bool(getattr(getattr(car_state, "cruiseState", None), "enabled", False)),
        "steer_fault_temporary": bool(getattr(car_state, "steerFaultTemporary", False)),
        "steer_fault_permanent": bool(getattr(car_state, "steerFaultPermanent", False)),
        "v_ego_mps": finite_or_none(car_state.vEgo),
        "a_ego_mps2": finite_or_none(car_state.aEgo),
        "requested_accel_mps2": finite_or_none(car_control.actuators.accel),
        "controller_accel_mps2": finite_or_none(car_output.actuatorsOutput.accel),
        "plan_source": str(plan.longitudinalPlanSource),
        "plan_accel_mps2": finite_or_none(plan.aTarget),
        "plan_should_stop": bool(plan.shouldStop),
        "plan_has_lead": bool(plan.hasLead),
        "lead_probability": finite_or_none(lead.prob) if lead is not None else None,
        "lead_distance_m": finite_or_none(lead.x[0]) if lead is not None and len(lead.x) else None,
        "model_leads": [{"probability": finite_or_none(candidate.prob),
                         "distance_m": finite_or_none(candidate.x[0]) if len(candidate.x) else None,
                         "lateral_m": finite_or_none(candidate.y[0]) if len(candidate.y) else None,
                         "speed_mps": finite_or_none(candidate.v[0]) if len(candidate.v) else None}
                        for candidate in model.leadsV3],
        "radar_lead_one": radar_lead_snapshot(radar.leadOne) if radar is not None else None,
        "radar_lead_two": radar_lead_snapshot(radar.leadTwo) if radar is not None else None,
        "message_status": {service: {"valid": valid.get(service), "alive": alive.get(service),
                                     "mono_time_ns": mono_time.get(service)} for service in services},
      }
    except (AttributeError, IndexError, KeyError, RuntimeError, TypeError, ValueError):
      self.latest_openpilot = {"available": False}

  def close(self, simulation_time_s=None):
    if self.report_file is None:
      return
    elapsed_s = None if self.start_time_s is None or simulation_time_s is None else simulation_time_s - self.start_time_s
    self._write({"type": "scenario_closed", "scene_time_s": elapsed_s})
    self.report_file.close()
    self.report_file = None
    if self._manifest_thread is not None:
      self._manifest_thread.join()

  def _prepare(self):
    self.report_file = RotatingReport(self.report_dir)
    self._write({
      "type": "scenario_started",
      "scene": "motorcycle_weave",
      "case": self.case,
      "seed": self.seed,
      "duration_s": self.duration_s,
    })
    collision_bp = self.world.get_blueprint_library().find("sensor.other.collision")
    sensor = self.world.spawn_actor(collision_bp, self.carla.Transform(), attach_to=self.ego_vehicle)
    sensor.listen(self._on_collision)
    self.actor_sink.append(sensor)

  def _write_manifest(self):
    self.report_dir.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[6]
    model_dir = root / "openpilot/selfdrive/modeld/models"
    artifact_names = ("driving_supercombo.onnx", "big_driving_supercombo.onnx")
    compiled = sorted((*model_dir.glob("driving_tinygrad.pkl*"),
                       *model_dir.glob("big_driving_tinygrad.pkl*")))
    source_paths = ("openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py",
                    "openpilot/tools/sim/bridge/common.py",
                    "openpilot/tools/sim/bridge/carla/carla_world.py",
                    "openpilot/selfdrive/controls/lib/longitudinal_planner.py",
                    "openpilot/selfdrive/controls/lib/vn_traffic_policy.py")
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True,
                              text=True, check=False, timeout=5)
    try:
      from openpilot.common.params import Params
      params = Params()
      captured_params = {key: param_value_text(value)
                         for key in ("ExperimentalMode", "LongitudinalPersonality")
                         if (value := params.get(key)) is not None}
    except (ImportError, OSError, RuntimeError):
      captured_params = {}
    manifest = {
      "schema_version": 1,
      "git_commit": revision.stdout.strip() if revision.returncode == 0 else None,
      "scene": "motorcycle_weave", "case": self.case, "seed": self.seed,
      "duration_s": self.duration_s, "command_argv": sys.argv,
      "carla_map": getattr(self, "carla_map_name", None),
      "carla_server_version": getattr(self, "carla_server_version", None),
      "params": captured_params,
      "vn_traffic_mode": os.environ.get("VN_TRAFFIC_MODE", "0"),
      "vn_traffic_profile": os.environ.get("VN_TRAFFIC_PROFILE", "balanced"),
      "source_sha256": {name: sha256_path(root / name) for name in source_paths},
      "onnx_sha256": {name: sha256_path(model_dir / name) for name in artifact_names},
      "compiled_artifact_sha256": {path.name: sha256_path(path) for path in compiled},
      "compiled_source_link_verified": False,
    }
    (self.report_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

  def _ego_speed(self):
    velocity = self.ego_vehicle.get_velocity()
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

  def _candidate_lanes(self):
    ego_lane = self.world.get_map().get_waypoint(
      self.ego_vehicle.get_location(), project_to_road=True, lane_type=self.carla.LaneType.Driving)
    if ego_lane is None:
      return None
    left_lane = self._driving_neighbor(ego_lane, "left")
    right_lane = self._driving_neighbor(ego_lane, "right")
    if left_lane is None or right_lane is None:
      return None
    try:
      for lane in (ego_lane, left_lane, right_lane):
        self._advance(lane, 80.0)
    except RuntimeError:
      return None
    return ego_lane, left_lane, right_lane, 80.0

  def _retire_actor(self, item, elapsed_s):
    actor = item["actor"]
    actor.destroy()
    self._actors.remove(item)
    self.actor_sink.remove(actor)
    self._write({"type": "cut_in_retired", "scene_time_s": elapsed_s, "actor_id": actor.id})

  def _vehicle_length_buffer(self, old_actor):
    motorcycle_half_length = old_actor["actor"].bounding_box.extent.x if old_actor not in (None, False) else 1.1
    return self.ego_vehicle.bounding_box.extent.x + motorcycle_half_length

  def _start_pass(self, event, ego_lane, left_lane, right_lane, staged_item):
    expected_source = right_lane if event.direction == "right_to_left" else left_lane
    actor = staged_item["actor"]
    source = self.world.get_map().get_waypoint(
      actor.get_location(), project_to_road=True, lane_type=self.carla.LaneType.Driving)
    if source is None or not same_lane_path(source, expected_source):
      return False
    middle = self._driving_neighbor(source, "left" if event.direction == "right_to_left" else "right")
    target = self._driving_neighbor(middle, "left" if event.direction == "right_to_left" else "right") if middle is not None else None
    if middle is None or target is None:
      return False
    try:
      self._advance(source, event.speed_mps * event.total_s + 20.0)
    except RuntimeError:
      return False
    self._background.remove(staged_item)
    self._actors.append(staged_item)
    actual_event = replace(event, spawn_center_distance_m=0.0)
    staged_item.update(event=actual_event, source=source, middle=middle, target=target,
                speed_integral=0.0, completed=False, progress_m=0.0,
                last_progress_time_s=None, entered_ego_lane_at=None, left_ego_lane_at=None)
    self._write({"type": "cut_in_started", "scene_time_s": event.start_time_s,
                 "actor_id": actor.id, "direction": event.direction,
                 "speed_mps": event.speed_mps, "merge_gap_m": event.merge_gap_m,
                 "entry_s": event.entry_s, "hold_s": event.hold_s, "exit_s": event.exit_s,
                 "from_staged_background": True})
    return True

  def _driving_neighbor(self, waypoint, side):
    candidate = waypoint.get_left_lane() if side == "left" else waypoint.get_right_lane()
    if candidate is None or candidate.lane_type != self.carla.LaneType.Driving:
      return None
    if candidate.road_id != waypoint.road_id or candidate.lane_id * waypoint.lane_id <= 0:
      return None
    return candidate

  def _advance(self, waypoint, distance_m):
    remaining = distance_m
    current = waypoint
    while remaining > 0.001:
      step = min(2.0, remaining)
      candidates = current.next(step)
      match = next((candidate for candidate in candidates if same_lane_path(candidate, waypoint)), None)
      if match is None:
        raise RuntimeError(f"motorcycle_weave lane ends before {distance_m:.1f} m")
      current = match
      remaining -= step
    return current

  def _target_location(self, item, elapsed_s, progress, *, lookahead_m):
    return self._target_pose(item, elapsed_s, progress, lookahead_m=lookahead_m)[0]

  def _control_progress(self, item):
    event = item["event"]
    distance = item["progress_m"]
    # The bike needs more preview on entry than exit. A full 3 m of exit
    # preview pulls it out of the ego lane before the reference path does.
    lateral_lookahead = (self.LOOKAHEAD_M if distance < event.speed_mps * event.entry_s else
                         self.EXIT_LATERAL_LOOKAHEAD_M)
    return pass_lateral_progress(event, (distance + lateral_lookahead) / event.speed_mps)

  def _target_pose(self, item, elapsed_s, progress, *, lookahead_m):
    event = item["event"]
    distance = event.spawn_center_distance_m + item["progress_m"] + lookahead_m
    source_transform = self._advance(item["source"], distance).transform
    middle_transform = self._advance(item["middle"], distance).transform
    target_transform = self._advance(item["target"], distance).transform
    if progress <= 0.5:
      source, target, fraction = source_transform.location, middle_transform.location, progress * 2.0
    else:
      source, target, fraction = middle_transform.location, target_transform.location, (progress - 0.5) * 2.0
    location = self.carla.Location(
      x=source.x + (target.x - source.x) * fraction,
      y=source.y + (target.y - source.y) * fraction,
      z=source.z + (target.z - source.z) * fraction,
    )
    return location, source_transform.rotation.yaw

  def _advance_actor_progress(self, item, simulation_time_s):
    previous_time = item["last_progress_time_s"]
    item["last_progress_time_s"] = simulation_time_s
    if previous_time is None:
      return
    event = item["event"]
    try:
      reference = self._advance(item["source"], event.spawn_center_distance_m + item["progress_m"])
    except RuntimeError:
      return
    velocity = item["actor"].get_velocity()
    item["progress_m"] = advance_path_distance(
      item["progress_m"], velocity_x=velocity.x, velocity_y=velocity.y,
      route_yaw_deg=reference.transform.rotation.yaw, dt_s=simulation_time_s - previous_time)

  def _track_lane_occupancy(self, item, elapsed_s):
    event = item["event"]
    try:
      middle = self._advance(item["middle"], event.spawn_center_distance_m + item["progress_m"])
    except RuntimeError:
      return
    position = item["actor"].get_location()
    _, lateral_offset = tracking_error_components(
      actual_x=position.x, actual_y=position.y,
      reference_x=middle.transform.location.x, reference_y=middle.transform.location.y,
      reference_yaw_deg=middle.transform.rotation.yaw)
    if abs(lateral_offset) <= 0.6 and item["entered_ego_lane_at"] is None:
      item["entered_ego_lane_at"] = elapsed_s
      gap_m = None
      if hasattr(item["actor"], "bounding_box") and hasattr(self.ego_vehicle, "bounding_box"):
        actor_transform = item["actor"].get_transform()
        ego_transform = self.ego_vehicle.get_transform()
        actor_polygon = [(vertex.x, vertex.y) for vertex in
                         item["actor"].bounding_box.get_world_vertices(actor_transform)]
        ego_polygon = [(vertex.x, vertex.y) for vertex in
                       self.ego_vehicle.bounding_box.get_world_vertices(ego_transform)]
        gap_m = polygon_clearance(actor_polygon, ego_polygon)
      self._write({"type": "cut_in_entered_ego_lane", "scene_time_s": elapsed_s,
                   "actor_id": item["actor"].id, "actual_path_distance_m": item["progress_m"],
                   "actual_gap_m": gap_m})
    elif abs(lateral_offset) > 0.6 and item["entered_ego_lane_at"] is not None and item["left_ego_lane_at"] is None:
      item["left_ego_lane_at"] = elapsed_s
      self._write({"type": "cut_in_left_ego_lane", "scene_time_s": elapsed_s,
                   "actor_id": item["actor"].id,
                   "actual_hold_s": elapsed_s - item["entered_ego_lane_at"]})

  def _apply_control(self, item, target):
    actor = item["actor"]
    transform = actor.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    dx, dy = target.x - location.x, target.y - location.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    steer = motorcycle_steer_command(local_x=local_x, local_y=local_y)
    item["last_steer"] = steer
    velocity = actor.get_velocity()
    speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    target_speed = item["speed_mps"] if "speed_mps" in item else item["event"].speed_mps
    throttle, brake, item["speed_integral"] = motorcycle_longitudinal_control(
      speed, item["speed_integral"], target_speed)
    actor.apply_control(self.carla.VehicleControl(
      throttle=throttle,
      steer=steer,
      brake=brake,
    ))

  def _on_collision(self, event):
    other = event.other_actor
    impulse = getattr(event, "normal_impulse", None)
    impulse_norm = (math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
                    if impulse is not None else None)
    timestamp = getattr(event, "timestamp", None)
    record = {
      "type": "collision",
      "scene_time_s": timestamp - self.start_time_s if timestamp is not None and self.start_time_s is not None else None,
      "carla_timestamp_s": timestamp,
      "frame": getattr(event, "frame", None),
      "other_actor_id": other.id,
      "other_actor_type": other.type_id,
      "impulse_norm": impulse_norm,
    }
    self._collisions.append(record)
    self._write(record)

  def _write(self, record):
    if self.report_file is not None:
      self.report_file.write(record)
