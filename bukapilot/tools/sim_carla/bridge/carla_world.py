"""CARLA world for the bukapilot bridge, ported from
openpilot/tools/sim/bridge/carla/carla_world.py.

Kept compatible with the CARLA 0.9.12 Python API pinned in
tools/sim/Dockerfile.bukapilot (no Client.load_world_if_different), and with
the legacy bridge's camera mount, which matches the 0.8.13 EON-era model.
"""
import math
import os
import queue
import time

import numpy as np

from tools.sim_carla.bridge.common import QueueMessage, QueueMessageType
from tools.sim_carla.lib.common import H, W, SimulatorState, World, vec3


class CarlaWorld(World):
  CAMERA_TIMEOUT = 5.0
  FIXED_DELTA_SECONDS = 0.05
  STEER_RATIO = 15.0  # legacy bridge value, close to the Civic's CP.steerRatio (15.38)
  # Legacy tools/sim/bridge.py mount (x forward of actor origin, z up), overridable.
  CAMERA_X = float(os.environ.get("CARLA_CAMERA_X", "0.8"))
  CAMERA_Z = float(os.environ.get("CARLA_CAMERA_Z", "1.13"))
  ROAD_FOV = 40.0

  def __init__(self, status_q, host, port, town, spawn_point, vehicle="vehicle.tesla.model3",
               low_quality=False, weather="ClearNoon"):
    super().__init__()
    import carla  # pylint: disable=import-error
    self.carla = carla
    self.status_q = status_q
    self.actors = []
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
      self.current_frame = self.world.tick()
      self._update_spectator()
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
    self.notify_reset()

  def close(self, reason: str):
    if self.closed:
      return
    self.closed = True
    self.status_q.put(QueueMessage(QueueMessageType.CLOSE_STATUS, reason))
    self.exit_event.set()
    for actor in reversed(self.actors):
      if actor.is_alive:
        if hasattr(actor, "stop"):
          actor.stop()
        actor.destroy()
    self.world.apply_settings(self.original_settings)
