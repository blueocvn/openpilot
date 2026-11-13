"""CARLA world for the bukapilot bridge, ported from
openpilot/tools/sim/bridge/carla/carla_world.py.

Kept compatible with the CARLA 0.9.12 Python API pinned in
tools/sim/Dockerfile.bukapilot (no Client.load_world_if_different), and with
the legacy bridge's camera mount, which matches the 0.8.13 EON-era model.
"""
import collections
import math
import os
import queue
import time

import numpy as np

from tools.sim.bridge.common import QueueMessage, QueueMessageType, lead_truth, nearest_ahead
from tools.sim.bridge.run_metrics import OBSTACLE_KINDS, project_half_length
from tools.sim.lib.common import H, W, SimulatorState, World, vec3


class CarlaWorld(World):
  CAMERA_TIMEOUT = 5.0
  FIXED_DELTA_SECONDS = 0.05
  STEER_RATIO = 15.0  # legacy bridge value, close to the Civic's CP.steerRatio (15.38)
  # Legacy tools/sim/bridge.py mount (x forward of actor origin, z up), overridable.
  CAMERA_X = float(os.environ.get("CARLA_CAMERA_X", "0.8"))
  CAMERA_Z = float(os.environ.get("CARLA_CAMERA_Z", "1.13"))
  ROAD_FOV = 40.0

  def __init__(self, status_q, host, port, town, spawn_point, vehicle="vehicle.tesla.model3",
               low_quality=False, weather="ClearNoon", motorcycle=None, obstacles=None):
    super().__init__()
    import carla  # pylint: disable=import-error
    self.carla = carla
    self.status_q = status_q
    self.actors = []
    self.traffic_manager = None
    self.motorcycles = []
    self.obstacles = []
    self.collisions = collections.deque(maxlen=32)  # (other actor type, impulse N*s), filled by the CARLA sensor thread
    self.tick_count = 0
    self.camera_queue = queue.Queue(maxsize=4)
    self.latest_gnss = None
    self.latest_imu = None
    self.current_frame = None
    self.closed = False

    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, f"connecting to CARLA at {host}:{port}"))
    self.client = carla.Client(host, port)
    # large maps can take well over 10 s to load the first time
    self.client.set_timeout(120.0)
    self.world = self._load_world(town)
    self.original_settings = self.world.get_settings()

    settings = self.world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = self.FIXED_DELTA_SECONDS
    settings.no_rendering_mode = False
    self.world.apply_settings(settings)
    self.world.set_weather(getattr(carla.WeatherParameters, weather))

    if low_quality:
      for layer in ("Foliage", "Buildings", "ParkedVehicles", "Props", "StreetLights", "Particles"):
        self.world.unload_map_layer(getattr(carla.MapLayer, layer))

    spawn_points = self.world.get_map().get_spawn_points()
    if not 0 <= spawn_point < len(spawn_points):
      raise ValueError(f"spawn point {spawn_point} out of range, {town} has 0..{len(spawn_points) - 1}")
    self.spawn_transform = spawn_points[spawn_point]

    vehicle_bp = self.world.get_blueprint_library().find(vehicle)
    if vehicle_bp.has_attribute("role_name"):
      vehicle_bp.set_attribute("role_name", "hero")
    self.vehicle = self.world.try_spawn_actor(vehicle_bp, self.spawn_transform)
    if self.vehicle is None:
      raise RuntimeError(f"CARLA spawn point {spawn_point} is occupied")
    self.actors.append(self.vehicle)
    self.world.tick()  # a freshly spawned actor has no valid transform until the server steps

    physics = self.vehicle.get_physics_control()
    self.max_wheel_angle = max(float(w.max_steer_angle) for w in physics.wheels)
    # CARLA scales steering down with speed to make manual driving forgiving.
    # A real EPS reaches the commanded angle at any speed: flatten the curve.
    physics.steering_curve = [carla.Vector2D(0.0, 1.0), carla.Vector2D(400.0, 1.0)]
    self.vehicle.apply_physics_control(physics)

    self._spawn_camera()
    self._spawn_sensor("sensor.other.gnss", lambda e: setattr(self, "latest_gnss", e))
    self._spawn_sensor("sensor.other.imu", lambda e: setattr(self, "latest_imu", e))
    self._spawn_sensor("sensor.other.collision", self._on_collision)
    if motorcycle:
      self._spawn_motorcycles(**motorcycle)
    if obstacles:
      self._spawn_obstacles(obstacles)
    self.spectator_mode = os.environ.get("CARLA_SPECTATOR_MODE", "chase")
    self._update_spectator()

    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))
    print(f"CARLA connected: {host}:{port}, map={town}, spawn={spawn_point}, vehicle={vehicle}", flush=True)

  def _load_world(self, town):
    world = self.client.get_world()
    current = world.get_map().name.split("/")[-1]
    if current != town:
      print(f"CARLA: loading {town} (was {current})", flush=True)
      world = self.client.load_world(town)
    return world

  @staticmethod
  def _free_port():
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.bind(("", 0))
      return s.getsockname()[1]

  MOTORCYCLE_MODELS = ("vehicle.kawasaki.ninja", "vehicle.yamaha.yzf", "vehicle.harley-davidson.low_rider")

  def _lane_waypoint(self, ego_wp, kind):
    """(waypoint at the ego position in the lane for `kind`, oncoming?) or None if there is no such lane.

    same: the ego lane. left/right: the adjacent lane going the same way. oncoming: the nearest lane going
    the other way (CARLA towns are right-hand traffic, so it is reached by hopping left).
    """
    driving = self.carla.LaneType.Driving
    if kind == "same":
      return ego_wp, False
    if kind in ("left", "right"):
      wp = ego_wp.get_left_lane() if kind == "left" else ego_wp.get_right_lane()
      if wp is not None and wp.lane_type == driving and (wp.lane_id > 0) == (ego_wp.lane_id > 0):
        return wp, False
      return None
    wp = ego_wp
    for _ in range(8):  # hop across lanes (and the median) until one drives the other way
      wp = wp.get_left_lane()
      if wp is None:
        return None
      if (wp.lane_id > 0) != (ego_wp.lane_id > 0) and wp.lane_type == driving:
        return wp, True
    return None

  def _spawn_motorcycles(self, specs, models=None):
    """Motorcycles around the ego car, driven by CARLA's Traffic Manager (no lane changes, ignoring lights).

    specs: list of (kind, gap_m, speed_kph), kind in same | left | right | oncoming; gap_m is metres ahead of
    the ego spawn point along the ego direction of travel. The TM runs in this process and needs
    synchronous mode, which is already on for the world.
    """
    library = self.world.get_blueprint_library()
    models = list(models or self.MOTORCYCLE_MODELS)
    ego_wp = self.world.get_map().get_waypoint(self.spawn_transform.location)
    # a fixed port (CARLA's default 8000) is often still held by a bridge from an earlier run: take a free one
    tm = self.client.get_trafficmanager(int(os.environ.get("CARLA_TM_PORT") or self._free_port()))
    tm.set_synchronous_mode(True)
    self.traffic_manager = tm

    for n, (kind, gap, speed_kph) in enumerate(specs):
      label = f"#{n} {kind}:{gap:g}:{speed_kph:g}"
      found = self._lane_waypoint(ego_wp, kind)
      if found is None:
        print(f"CARLA: motorcycle {label}: no {kind} lane at this spawn point, skipped", flush=True)
        continue
      lane_wp, oncoming = found
      bp = library.find(models[n % len(models)])
      if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", f"motorcycle_{n}")
      bike = transform = None
      for extra in range(0, 30, 5):  # the first free spot at or beyond the requested gap
        d = float(gap) + extra
        # "ahead of the ego car" is backwards along an oncoming lane
        candidates = lane_wp.previous(d) if oncoming else lane_wp.next(d)
        if not candidates:
          continue
        transform = candidates[0].transform
        transform.location.z += 0.5  # drop it onto the road instead of spawning inside it
        bike = self.world.try_spawn_actor(bp, transform)
        if bike is not None:
          break
      if bike is None:
        print(f"CARLA: motorcycle {label}: could not spawn, skipped", flush=True)
        continue
      self.actors.append(bike)
      self.motorcycles.append({"actor": bike, "transform": transform, "speed_kph": float(speed_kph), "label": label})

    self.world.tick()
    for m in self.motorcycles:
      bike = m["actor"]
      bike.set_autopilot(True, tm.get_port())
      tm.auto_lane_change(bike, False)
      tm.ignore_lights_percentage(bike, 100)
      tm.ignore_signs_percentage(bike, 100)
    self._update_motorcycle_speeds()
    print(f"CARLA: {len(self.motorcycles)}/{len(specs)} motorcycles: " + ", ".join(m["label"] for m in self.motorcycles),
          flush=True)

  @staticmethod
  def _yaw_change(a, b):
    return abs((a.transform.rotation.yaw - b.transform.rotation.yaw + 180.0) % 360.0 - 180.0)

  def _straight_path(self, start_wp, length_m, step=1.0):
    """Waypoints every `step` m along the road ahead, taking the straightest branch at every junction."""
    path = [start_wp]
    while (len(path) - 1) * step < length_m:
      ahead = path[-1].next(step)
      if not ahead:
        break
      path.append(min(ahead, key=lambda wp: self._yaw_change(wp, path[-1])))
    return path

  def _spawn_obstacles(self, specs):
    """Obstacles in the ego lane, `distance_m` ahead of the ego spawn point along the straight road.

    specs: list of (kind, distance_m, speed_kph), kind a key of OBSTACLE_KINDS. Vehicles rest on the road with the
    hand brake on; props are static. A motorcycle with a speed is driven by this class, not by the Traffic Manager:
    it follows the straight road ahead at that constant speed and never turns at a junction (see _move_obstacles).
    """
    library = self.world.get_blueprint_library()
    ego_wp = self.world.get_map().get_waypoint(self.spawn_transform.location)
    path = self._straight_path(ego_wp, max(d for _, d, _ in specs) + 3000.0)
    for n, (kind, distance, speed_kph) in enumerate(specs):
      blueprint, yaw_offset = OBSTACLE_KINDS[kind]
      label = f"#{n} {kind}:{distance:g}" + (f":{speed_kph:g}" if speed_kph else "")
      if int(distance) >= len(path):
        print(f"CARLA: obstacle {label}: the road ends before that distance, skipped", flush=True)
        continue
      transform = path[int(distance)].transform
      transform.rotation.yaw += yaw_offset
      transform.location.z += 0.3
      bp = library.find(blueprint)
      if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", f"obstacle_{n}")
      actor = self.world.try_spawn_actor(bp, transform)
      if actor is None:
        print(f"CARLA: obstacle {label}: could not spawn, skipped", flush=True)
        continue
      self.actors.append(actor)
      entry = {"actor": actor, "yaw_offset": yaw_offset, "label": label}
      if speed_kph:
        entry.update(speed=speed_kph / 3.6, s=float(distance), velocity=(0.0, 0.0), path=path)
      elif actor.type_id.startswith("vehicle."):
        actor.apply_control(self.carla.VehicleControl(hand_brake=True))
      self.obstacles.append(entry)
    for _ in range(15):   # let a driven vehicle settle on the road, then take it off physics
      self.world.tick()
    for entry in self.obstacles:
      if "speed" in entry:
        wp = entry["path"][int(entry["s"])]
        entry["z_offset"] = entry["actor"].get_location().z - wp.transform.location.z
        entry["actor"].set_simulate_physics(False)
    print(f"CARLA: {len(self.obstacles)}/{len(specs)} obstacles: " + ", ".join(o["label"] for o in self.obstacles), flush=True)

  def _move_obstacles(self):
    """Advance the driven obstacles one simulation step along their straight path at constant speed."""
    for entry in self.obstacles:
      if "speed" not in entry or not entry["actor"].is_alive:
        continue
      path = entry["path"]
      entry["s"] = min(entry["s"] + entry["speed"] * self.FIXED_DELTA_SECONDS, len(path) - 2.0)
      i = int(entry["s"])
      frac = entry["s"] - i
      a, b = path[i].transform, path[i + 1].transform
      location = self.carla.Location(a.location.x + (b.location.x - a.location.x) * frac,
                                     a.location.y + (b.location.y - a.location.y) * frac,
                                     a.location.z + (b.location.z - a.location.z) * frac + entry["z_offset"])
      entry["actor"].set_transform(self.carla.Transform(location, a.rotation))
      yaw = math.radians(a.rotation.yaw)
      entry["velocity"] = (entry["speed"] * math.cos(yaw), entry["speed"] * math.sin(yaw))

  def _update_motorcycle_speeds(self):
    # The TM speed is "percent below the road's speed limit": recompute it from the limit where each bike is now.
    for m in self.motorcycles:
      bike = m["actor"]
      if not bike.is_alive:
        continue
      limit = float(bike.get_speed_limit()) or 50.0  # km/h
      pct = float(np.clip((1.0 - m["speed_kph"] / limit) * 100.0, -100.0, 100.0))
      self.traffic_manager.vehicle_percentage_speed_difference(bike, pct)

  def _on_collision(self, event):
    impulse = event.normal_impulse
    self.collisions.append((event.other_actor.type_id, math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)))

  def consume_collisions(self):
    """Collisions of the ego car since the last call, as (other actor type, impulse). Logged, never sent to openpilot."""
    out = []
    while self.collisions:
      out.append(self.collisions.popleft())
    return out

  def _truth_others(self, ego):
    """(xy, velocity xy, half-length along the ego heading) of every spawned motorcycle and obstacle."""
    ego_yaw = ego.rotation.yaw
    others = []
    for entry in self.motorcycles + self.obstacles:
      actor = entry["actor"]
      if not actor.is_alive:
        continue
      location = actor.get_location()
      if "velocity" in entry:   # a driven obstacle is moved by set_transform: CARLA does not report its speed
        velocity_xy = entry["velocity"]
      else:
        velocity = actor.get_velocity()
        velocity_xy = (velocity.x, velocity.y)
      extent = actor.bounding_box.extent
      half = project_half_length(extent.x, extent.y, actor.get_transform().rotation.yaw - ego_yaw)
      others.append(((location.x, location.y), velocity_xy, half))
    return others

  def truth_lead(self):
    """Ground truth of the nearest spawned object ahead in the ego lane: (bumper gap m, speed along ego heading m/s).
    Logged, never sent to openpilot."""
    if not self.motorcycles and not self.obstacles:
      return None
    ego = self.vehicle.get_transform()
    forward = ego.get_forward_vector()
    return lead_truth((ego.location.x, ego.location.y), (forward.x, forward.y),
                      self.vehicle.bounding_box.extent.x, self._truth_others(ego))

  def truth_lateral(self, corridor_m=6.0):
    """Lateral offset (m, left positive, as openpilot's yRel) of the nearest spawned object ahead within a wide
    corridor: it stays defined while the ego steers out of the obstacle's lane. Logged, never sent to openpilot."""
    if not self.motorcycles and not self.obstacles:
      return None
    ego = self.vehicle.get_transform()
    forward = ego.get_forward_vector()
    found = nearest_ahead((ego.location.x, ego.location.y), (forward.x, forward.y),
                          self.vehicle.bounding_box.extent.x, self._truth_others(ego), corridor_m)
    return None if found is None else found[2]

  def ego_pose(self):
    """Ego pose for the frame recorder: position, heading and pitch in the CARLA frame."""
    t = self.vehicle.get_transform()
    return {"x": t.location.x, "y": t.location.y, "z": t.location.z,
            "yaw": t.rotation.yaw, "pitch": t.rotation.pitch, "roll": t.rotation.roll}

  def _sensor_transform(self):
    return self.carla.Transform(self.carla.Location(x=self.CAMERA_X, z=self.CAMERA_Z))

  @staticmethod
  def _put_latest(q, item):
    try:
      q.put_nowait(item)
    except queue.Full:
      try:
        q.get_nowait()
      except queue.Empty:
        pass
      q.put_nowait(item)

  def _spawn_camera(self):
    bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(W))
    bp.set_attribute("image_size_y", str(H))
    bp.set_attribute("fov", str(self.ROAD_FOV))
    bp.set_attribute("sensor_tick", str(self.FIXED_DELTA_SECONDS))
    if bp.has_attribute("enable_postprocess_effects"):
      bp.set_attribute("enable_postprocess_effects", "true")  # exposure / tone mapping
    sensor = self.world.spawn_actor(bp, self._sensor_transform(), attach_to=self.vehicle)
    sensor.listen(lambda image: self._put_latest(self.camera_queue, image))
    self.actors.append(sensor)

  def _spawn_sensor(self, blueprint, callback):
    bp = self.world.get_blueprint_library().find(blueprint)
    sensor = self.world.spawn_actor(bp, self._sensor_transform(), attach_to=self.vehicle)
    sensor.listen(callback)
    self.actors.append(sensor)

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    steer = -float(steer_angle) / max(self.max_wheel_angle * self.STEER_RATIO, 1.0)
    self.vehicle.apply_control(self.carla.VehicleControl(
      throttle=float(np.clip(throttle_out, 0.0, 1.0)),
      steer=float(np.clip(steer, -1.0, 1.0)),
      brake=float(np.clip(brake_out, 0.0, 1.0)),
      reverse=False,
      hand_brake=False,
    ))

  def _update_spectator(self):
    """Chase view behind the ego car in the CARLA window (CARLA_SPECTATOR_MODE=free to disable)."""
    if self.spectator_mode == "free":
      return
    transform = self.vehicle.get_transform()
    forward = transform.get_forward_vector()
    location = self.carla.Location(x=transform.location.x - 8.0 * forward.x,
                                   y=transform.location.y - 8.0 * forward.y,
                                   z=transform.location.z + 3.5)
    rotation = self.carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw)
    self.world.get_spectator().set_transform(self.carla.Transform(location, rotation))

  def tick(self):
    try:
      self._move_obstacles()
      self.current_frame = self.world.tick()
      self._update_spectator()
      self.tick_count += 1
      if self.motorcycles and self.tick_count % 10 == 0:
        self._update_motorcycle_speeds()
    except RuntimeError as exc:
      self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, str(exc)))
      self.exit_event.set()

  def read_sensors(self, state: SimulatorState):
    velocity = self.vehicle.get_velocity()
    state.velocity = vec3(float(velocity.x), float(velocity.y), float(velocity.z))
    # Report the wheel angle actually reached instead of echoing the command
    # (the legacy bridge echoed it), so openpilot sees any rack shortfall.
    fl = self.vehicle.get_wheel_steer_angle(self.carla.VehicleWheelLocation.FL_Wheel)
    fr = self.vehicle.get_wheel_steer_angle(self.carla.VehicleWheelLocation.FR_Wheel)
    state.steering_angle = -0.5 * (float(fl) + float(fr)) * self.STEER_RATIO

    transform = self.vehicle.get_transform()
    state.bearing = math.radians(transform.rotation.yaw)
    state.imu.bearing = transform.rotation.yaw % 360.0
    if self.latest_gnss is not None:
      state.gps.latitude = float(self.latest_gnss.latitude)
      state.gps.longitude = float(self.latest_gnss.longitude)
      state.gps.altitude = float(self.latest_gnss.altitude)
    else:
      state.gps.from_xy((transform.location.x, transform.location.y))

    if self.latest_imu is not None:
      acc = self.latest_imu.accelerometer
      gyro = self.latest_imu.gyroscope
      # CARLA IMU is body frame (UE left-handed: x fwd, y right, z up).
      # locationd applies meas = [-v2, -v1, -v0] to reach the device frame
      # (x fwd, y right, z down); that remap is self-inverse, so pre-apply it:
      #   accel = (ax, ay, -az), gyro = (-gx, -gy, gz)
      state.imu.accelerometer = vec3(float(acc.z), float(-acc.y), float(-acc.x))
      state.imu.gyroscope = vec3(float(-gyro.z), float(gyro.y), float(gyro.x))
      state.imu.bearing = math.degrees(float(self.latest_imu.compass)) % 360.0
    state.valid = True

  def _camera_for_frame(self):
    deadline = time.monotonic() + self.CAMERA_TIMEOUT
    latest = None
    while time.monotonic() < deadline:
      try:
        image = self.camera_queue.get(timeout=max(deadline - time.monotonic(), 0.01))
      except queue.Empty:
        break
      latest = image
      if self.current_frame is None or image.frame >= self.current_frame:
        return image
    if latest is None:
      raise TimeoutError(f"timed out waiting for road camera frame {self.current_frame}")
    return latest

  def read_cameras(self):
    try:
      road = self._camera_for_frame()
      self.road_bgra_image = np.frombuffer(road.raw_data, dtype=np.uint8).reshape((road.height, road.width, 4))
      self.image_lock.release()
    except (TimeoutError, ValueError) as exc:
      self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, str(exc)))
      self.exit_event.set()

  def reset(self):
    self.vehicle.apply_control(self.carla.VehicleControl())
    self.vehicle.set_target_velocity(self.carla.Vector3D())
    self.vehicle.set_target_angular_velocity(self.carla.Vector3D())
    self.vehicle.set_transform(self.spawn_transform)
    for m in self.motorcycles:
      if m["actor"].is_alive:
        m["actor"].set_target_velocity(self.carla.Vector3D())
        m["actor"].set_transform(m["transform"])
    self.notify_reset()

  def close(self, reason: str):
    if self.closed:
      return
    self.closed = True
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()
    if self.traffic_manager is not None:
      self.traffic_manager.set_synchronous_mode(False)
    # Stop every sensor before destroying anything: a collision event that names an already destroyed actor
    # aborts the whole process from CARLA's own thread ("trying to operate on a destroyed actor").
    for actor in self.actors:
      if actor.is_alive and hasattr(actor, "stop"):
        actor.stop()
    for actor in reversed(self.actors):
      if actor.is_alive:
        actor.destroy()
    self.world.apply_settings(self.original_settings)
