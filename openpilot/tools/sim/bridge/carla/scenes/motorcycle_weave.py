"""Deterministic schedule and CARLA runtime for motorcycle cut-ins."""

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
import random
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
    hold_s = self.rng.uniform(1.5, 2.0)
    exit_s = self.rng.uniform(1.15, 1.55)
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
                          32.0 + 10.0 * (self.index % 4) + self.rng.uniform(0.0, 3.0),
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
  PROGRESS_LOOKAHEAD_M = 6.0

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
    root = Path(report_dir) if report_dir else Path(".carla/reports")
    self.report_dir = root / f"motorcycle-weave-{time.strftime('%Y%m%d-%H%M%S')}-{time.time_ns() % 1_000_000_000:09d}"
    self.report_file = None

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
      control_progress = pass_lateral_progress(event, pass_elapsed_s + self.PROGRESS_LOOKAHEAD_M / event.speed_mps)
      try:
        target = self._target_location(item, pass_elapsed_s, control_progress, lookahead_m=self.LOOKAHEAD_M)
      except RuntimeError:
        item["actor"].apply_control(self.carla.VehicleControl(throttle=0.0, brake=1.0))
        continue
      self._apply_control(item, target)
      if pass_elapsed_s >= event.total_s and not item["completed"]:
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
    old_actor = self._reusable_actor(direction)
    if any(not item["completed"] for item in self._actors):
      return
    event = self.scheduler.propose(elapsed_s, ego_speed_mps=self._ego_speed(), available_road_m=available_road_m,
                                   actor_available=old_actor is not False,
                                   vehicle_length_buffer_m=self._vehicle_length_buffer(old_actor))
    if event is not None and self._start_pass(event, ego_lane, left_lane, right_lane, old_actor):
      self.scheduler.commit(event)

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
        target = self._advance(item["source"], item["distance_m"] +
                               item["speed_mps"] * (elapsed_s - item["start_s"]) + self.LOOKAHEAD_M).transform.location
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
                             "start_s": elapsed_s, "speed_mps": slot.speed_mps, "speed_integral": 0.0})
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
      self._write({
        "type": "control_sample", "scene_time_s": elapsed_s,
        "ego_speed_mps": math.sqrt(ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2),
        "ego_brake": float(self.ego_vehicle.get_control().brake),
        "openpilot": self.latest_openpilot,
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
          item, pass_elapsed_s, pass_lateral_progress(event, pass_elapsed_s), lookahead_m=0.0,
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
      actors.append({
        "id": actor.id,
        "role": "cut_in",
        "direction": item["event"].direction,
        "phase": pass_lateral_progress(event, pass_elapsed_s),
        "target_speed_mps": event.speed_mps,
        "position_m": [transform.location.x, transform.location.y, transform.location.z],
        "speed_mps": math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
        "tracking_error_m": math.hypot(transform.location.x - target.x, transform.location.y - target.y),
        "longitudinal_tracking_error_m": longitudinal_error,
        "lateral_tracking_error_m": lateral_error,
        "distance_to_ego_m": polygon_clearance(ego_footprint, actor_footprint),
      })
    for item in self._background:
      actor = item["actor"]
      if not actor.is_alive:
        continue
      transform = actor.get_transform()
      velocity = actor.get_velocity()
      actors.append({
        "id": actor.id, "role": "background",
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
      self.latest_openpilot = {
        "available": True,
        "active": bool(selfdrive_state.active),
        "long_active": bool(car_control.longActive),
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

  def _reusable_actor(self, direction):
    candidates = [item for item in self._actors if item["event"].direction == direction]
    if not candidates:
      return None
    item = candidates[0]
    if not item["completed"] or not item["actor"].is_alive:
      return False
    ego = self.ego_vehicle.get_transform()
    actor = item["actor"].get_transform()
    distance = ego_relative_longitudinal(
      ego.location.x, ego.location.y, actor.location.x, actor.location.y, ego.rotation.yaw)
    return item if distance < -20.0 else False

  def _retire_actor(self, item, elapsed_s):
    actor = item["actor"]
    actor.destroy()
    self._actors.remove(item)
    self.actor_sink.remove(actor)
    self._write({"type": "cut_in_retired", "scene_time_s": elapsed_s, "actor_id": actor.id})

  def _vehicle_length_buffer(self, old_actor):
    motorcycle_half_length = old_actor["actor"].bounding_box.extent.x if old_actor not in (None, False) else 1.1
    return self.ego_vehicle.bounding_box.extent.x + motorcycle_half_length

  def _start_pass(self, event, ego_lane, left_lane, right_lane, old_actor):
    source, target = (right_lane, left_lane) if event.direction == "right_to_left" else (left_lane, right_lane)
    try:
      transform = self._advance(source, event.spawn_center_distance_m).transform
    except RuntimeError:
      return False
    transform.location.z += 0.15
    if not self._spawn_clear(transform):
      return False
    if old_actor is None:
      blueprint = self.world.get_blueprint_library().find("vehicle.vespa.zx125")
      actor = self.world.try_spawn_actor(blueprint, transform)
      if actor is None:
        return False
      self.actor_sink.append(actor)
      item = {"actor": actor}
      self._actors.append(item)
    else:
      item = old_actor
      actor = item["actor"]
      # Reuse only after this actor is well behind ego; never teleport an actor
      # still visible in front of the road camera.
      actor.set_transform(transform)
    forward = transform.get_forward_vector()
    actor.set_target_velocity(self.carla.Vector3D(
      x=forward.x * event.speed_mps, y=forward.y * event.speed_mps, z=0.0))
    item.update(event=event, source=source, middle=ego_lane, target=target,
                speed_integral=0.0, completed=False)
    self._write({"type": "cut_in_started", "scene_time_s": event.start_time_s,
                 "actor_id": actor.id, "direction": event.direction,
                 "speed_mps": event.speed_mps, "merge_gap_m": event.merge_gap_m,
                 "entry_s": event.entry_s, "hold_s": event.hold_s, "exit_s": event.exit_s})
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

  def _target_pose(self, item, elapsed_s, progress, *, lookahead_m):
    event = item["event"]
    distance = trajectory_distance(event.spawn_center_distance_m, event.speed_mps,
                                   elapsed_s, lookahead_m=lookahead_m)
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

  def _apply_control(self, item, target):
    actor = item["actor"]
    transform = actor.get_transform()
    location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    dx, dy = target.x - location.x, target.y - location.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    steer = motorcycle_steer_command(local_x=local_x, local_y=local_y)
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
    # The CARLA Vespa needs several seconds of full throttle to reach 5 m/s;
    # during that ramp it falls metres behind the planned cut-in position.
    # Keep the adversarial traffic at its seeded speed while preserving
    # CARLA steering and collision physics.
    forward = transform.get_forward_vector()
    actor.set_target_velocity(self.carla.Vector3D(
      x=forward.x * target_speed,
      y=forward.y * target_speed,
      z=0.0,
    ))

  def _on_collision(self, event):
    other = event.other_actor
    self._collisions.append({"other_actor_id": other.id, "other_actor_type": other.type_id})

  def _write(self, record):
    if self.report_file is not None:
      self.report_file.write(record)
