import math
import os
import queue
import time

import numpy as np

from openpilot.tools.sim.bridge.common import QueueMessage, QueueMessageType
from openpilot.tools.sim.lib.camerad import H, W
from openpilot.tools.sim.lib.common import SimulatorState, World, vec3


def wsl_host():
  """Return the Windows host address from WSL2's default route."""
  try:
    with open("/proc/net/route") as routes:
      for line in routes:
        fields = line.split()
        if len(fields) > 2 and fields[1] == "00000000":
          gateway = bytes.fromhex(fields[2])
          return ".".join(str(x) for x in gateway[::-1])
  except OSError:
    pass
  return "127.0.0.1"


class CarlaWorld(World):
  CAMERA_TIMEOUT = 5.0
  FIXED_DELTA_SECONDS = 0.05

  def __init__(self, status_q, host, port, town, spawn_point, dual_camera=False, high_quality=False):
    super().__init__(dual_camera)
    # Keep CARLA optional for users of the MetaDrive backend.
    import carla
    self.carla = carla
    self.status_q = status_q
    self.actors = []
    self.camera_queues = {}
    self.latest_gnss = None
    self.latest_imu = None
    self.current_frame = None
    self.closed = False

    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, f"connecting to CARLA at {host}:{port}"))
    self.client = carla.Client(host, port)
    # Large optimized maps can take over 20 seconds to load on their first run.
    self.client.set_timeout(120.0)
    self.world = self.client.load_world_if_different(town) or self.client.get_world()
    self.original_settings = self.world.get_settings()

    settings = self.world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = self.FIXED_DELTA_SECONDS
    settings.no_rendering_mode = False
    self.world.apply_settings(settings)
    self.world.set_weather(carla.WeatherParameters.ClearNoon)

    self.spawn_points = self.world.get_map().get_spawn_points()
    if not self.spawn_points:
      raise RuntimeError(f"CARLA map {town} has no spawn points")
    self.spawn_transform = self.spawn_points[spawn_point % len(self.spawn_points)]

    vehicle_bp = self.world.get_blueprint_library().find("vehicle.tesla.model3")
    if vehicle_bp.has_attribute("role_name"):
      vehicle_bp.set_attribute("role_name", "hero")
    self.vehicle = self.world.try_spawn_actor(vehicle_bp, self.spawn_transform)
    if self.vehicle is None:
      raise RuntimeError(f"CARLA spawn point {spawn_point} is occupied")
    self.actors.append(self.vehicle)
    # In synchronous mode a freshly spawned actor reports an unset transform
    # until the server steps, which would make world->vehicle conversions below
    # resolve against an identity frame.
    self.world.tick()

    physics = self.vehicle.get_physics_control()
    self.max_wheel_angle = max(float(w.max_steer_angle) for w in physics.wheels)
    self.steer_ratio = 12.0
    # CARLA scales steering down as speed rises (stock curve: 1.0 at 0 km/h to
    # 0.7 at 120 km/h) to make manual driving forgiving. A real EPS actuates to
    # the commanded angle at any speed, and openpilot's angle controller is pure
    # feedforward with no loop to claw back the missing authority, so flatten it.
    physics.steering_curve = [self.carla.Vector2D(0.0, 1.0), self.carla.Vector2D(400.0, 1.0)]
    self.vehicle.apply_physics_control(physics)
    # Axle offsets are fixed for the rigid body, so resolve them once at spawn.
    self._front_axle, self._rear_axle = self._axle_offsets()
    # CARLA renders debug shapes into the RGB sensors as well as the spectator
    # view, so leaving these on feeds the markers straight into modeld's input.
    # Opt in only for a visual mount check, with openpilot's output ignored.
    self.debug_markers = os.environ.get("CARLA_DEBUG_MARKERS", "0") != "0"
    # chase (default): fixed camera behind the car. free: spectator is fully
    # manual, never touched. orbit: position re-centers on the car every tick
    # while rotation is read back from whatever the mouse last dragged it to,
    # so dragging orbits the view around the car instead of panning in place.
    self.spectator_mode = os.environ.get("CARLA_SPECTATOR_MODE", "chase")
    self._log_geometry()
    # FOVs are not free parameters: they must match the focal lengths openpilot
    # assumes for this device, or the model misreads distance. For a 1928 px
    # wide frame, fcam's 2648.0 focal implies 40.0 deg and ecam's 567.0 implies
    # 119.07 deg (fov = 2*atan(W / (2*focal))). See DEVICE_CAMERAS in
    # common/transformations/camera.py.
    self._spawn_camera("road", 40.0, high_quality)
    if dual_camera:
      self._spawn_camera("wide", 119.07, high_quality)
    self._spawn_gnss()
    self._spawn_imu()
    self._update_spectator()

    self.status_q.put(QueueMessage(QueueMessageType.START_STATUS, "started"))
    print(f"CARLA connected: {host}:{port}, map={town}, spawn={spawn_point}")

  # Approximate a comma three without putting the lens inside the car. Height
  # matches what openpilot expects: calibrationd seeds HEIGHT_INIT at 1.22 m and
  # the MetaDrive bridge mounts its C3_POSITION at the same 1.22 m.
  #
  # A real device sits just behind the rearview mirror, near x=0.0 here. CARLA
  # renders the ego mesh into its own attached cameras, so some of the car is
  # always in frame, and what obstructs depends on where the lens sits: at x<=0
  # the mirror clips the top of the road camera and the wide camera picks up
  # cabin interior, while x>=0.4 is visually clean.
  #
  # A clean frame is not what drives well, though. Pushing the mount forward to
  # x=1.2 clears the body but measured WORSE over comparable road -- the
  # predicted path's frame-to-frame jitter at 30 m rose 0.123 -> 0.171 m
  # (p95 0.429 -> 0.661) -- and driving is better still back at 0.0/-0.4. The
  # lever arm ahead of the rear axle matters more than the self-occlusion, so
  # keep this short and re-measure before trading it away.
  #
  # Height stays at the 1.22 m openpilot assumes (calibrationd HEIGHT_INIT).
  # Raising the mount would be worse than moving it forward: the model infers
  # ground distance from the depression angle, so a camera high by a factor k
  # makes everything read 1/k times as far, i.e. it turns in early.
  # Verify with CARLA_DEBUG_MARKERS=1 before changing these.
  CAMERA_X, CAMERA_Z = 0.4, 1.22

  def _sensor_transform(self):
    return self.carla.Transform(self.carla.Location(x=self.CAMERA_X, z=self.CAMERA_Z))

  def _local_from_world(self, world_location, inverse_matrix):
    """Convert a world-space location into vehicle-local meters."""
    pt = np.array([world_location.x, world_location.y, world_location.z, 1.0])
    return (inverse_matrix @ pt)[:3]

  def _axle_offsets(self):
    """Return (front, rear) axle centers in vehicle-local meters.

    CARLA parents sensors to the actor origin, which sits near the middle of the
    body rather than at the rear axle that a kinematic vehicle model references.
    WheelPhysicsControl.position is a world-space point in centimeters, so it has
    to be pulled back into the vehicle frame before it can be compared against
    the sensor transform.
    """
    inv = np.array(self.vehicle.get_transform().get_inverse_matrix())
    wheels = []
    for w in self.vehicle.get_physics_control().wheels:
      loc = self.carla.Location(x=w.position.x / 100.0, y=w.position.y / 100.0, z=w.position.z / 100.0)
      wheels.append(self._local_from_world(loc, inv))
    # CARLA orders wheels front-left, front-right, rear-left, rear-right.
    return (wheels[0] + wheels[1]) / 2.0, (wheels[2] + wheels[3]) / 2.0

  def _log_geometry(self):
    """Report where the camera sits relative to the body and axles.

    openpilot's calibration only models device height, so any forward mount
    offset is baked into what the model sees. Print the real numbers to make the
    camera's lever arm ahead of each axle checkable against the car.
    """
    bb = self.vehicle.bounding_box
    cam = self._sensor_transform().location
    front_x, rear_x = self._front_axle[0], self._rear_axle[0]
    print("CARLA ego geometry (vehicle-local meters, +x forward of actor origin):")
    print(f"  body: length={2 * bb.extent.x:.2f} width={2 * bb.extent.y:.2f} bbox center x={bb.location.x:.2f}")
    print(f"  front axle x={front_x:+.2f}  rear axle x={rear_x:+.2f}  wheelbase={front_x - rear_x:.2f}")
    print(f"  camera x={cam.x:+.2f} z={cam.z:.2f}")
    print(f"  camera ahead of rear axle:  {cam.x - rear_x:.2f} m")
    print(f"  camera ahead of front axle: {cam.x - front_x:.2f} m")

  def _draw_debug_markers(self):
    """Draw camera and axle reference points in the CARLA spectator view."""
    debug = self.world.debug
    transform = self.vehicle.get_transform()
    life = self.FIXED_DELTA_SECONDS * 3
    red, blue, green, yellow = ((255, 40, 40), (60, 120, 255), (40, 220, 80), (250, 210, 40))

    def world_pt(x, z, y=0.0):
      # Build a fresh Location per call: Transform.transform() maps in place.
      return transform.transform(self.carla.Location(x=float(x), y=float(y), z=float(z)))

    def mark(location, label, rgb, size=0.08):
      color = self.carla.Color(*rgb)
      debug.draw_point(location, size=size, color=color, life_time=life)
      label_at = self.carla.Location(x=location.x, y=location.y, z=location.z + 0.3)
      debug.draw_string(label_at, label, color=color, life_time=life)

    cam_local = self._sensor_transform().location
    camera = world_pt(cam_local.x, cam_local.z)
    mark(camera, "camera", red, size=0.12)
    # Ground track of the camera, the rear axle, and the span between them. The
    # gap is the lever arm between what the model sees and where the car pivots.
    cam_ground = world_pt(cam_local.x, 0.05)
    rear_ground = world_pt(self._rear_axle[0], 0.05)
    front_ground = world_pt(self._front_axle[0], 0.05)
    mark(cam_ground, f"cam+{cam_local.x - self._rear_axle[0]:.2f}m", red, size=0.06)
    mark(rear_ground, "rear axle", green, size=0.06)
    mark(front_ground, "front axle", yellow, size=0.06)
    mark(world_pt(0.0, 0.05), "origin", blue, size=0.06)
    debug.draw_line(rear_ground, cam_ground, thickness=0.04, color=self.carla.Color(*red), life_time=life)
    # Camera boresight, so any unintended pitch/yaw of the mount is visible.
    debug.draw_arrow(camera, world_pt(cam_local.x + 2.5, cam_local.z),
                     thickness=0.03, arrow_size=0.12, color=self.carla.Color(*red), life_time=life)

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

  def _spawn_camera(self, name, fov, high_quality):
    bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(W))
    bp.set_attribute("image_size_y", str(H))
    bp.set_attribute("fov", str(fov))
    bp.set_attribute("sensor_tick", str(self.FIXED_DELTA_SECONDS))
    if bp.has_attribute("enable_postprocess_effects"):
      # Tone mapping and eye adaptation live behind this flag. Without them the
      # sensor hands back the raw linear render, which on a ClearNoon map still
      # measures ~37/255 mean with the road almost black and lane markings
      # barely separable -- nothing like the exposed footage the driving model
      # was trained on. Keep it on regardless of high_quality, which is meant to
      # trade frame rate for scene detail rather than to disable exposure.
      bp.set_attribute("enable_postprocess_effects", "true")
    sensor = self.world.spawn_actor(bp, self._sensor_transform(), attach_to=self.vehicle)
    frames = queue.Queue(maxsize=4)
    sensor.listen(lambda image, q=frames: self._put_latest(q, image))
    self.camera_queues[name] = frames
    self.actors.append(sensor)

  def _spawn_gnss(self):
    bp = self.world.get_blueprint_library().find("sensor.other.gnss")
    sensor = self.world.spawn_actor(bp, self._sensor_transform(), attach_to=self.vehicle)
    sensor.listen(lambda event: setattr(self, "latest_gnss", event))
    self.actors.append(sensor)

  def _spawn_imu(self):
    bp = self.world.get_blueprint_library().find("sensor.other.imu")
    sensor = self.world.spawn_actor(bp, self._sensor_transform(), attach_to=self.vehicle)
    sensor.listen(lambda event: setattr(self, "latest_imu", event))
    self.actors.append(sensor)

  def apply_controls(self, steer_angle, throttle_out, brake_out):
    steer = -float(steer_angle) / max(self.max_wheel_angle * self.steer_ratio, 1.0)
    control = self.carla.VehicleControl(
      throttle=float(np.clip(throttle_out, 0.0, 1.0)),
      steer=float(np.clip(steer, -1.0, 1.0)),
      brake=float(np.clip(brake_out, 0.0, 1.0)),
      reverse=False,
      hand_brake=False,
    )
    self.vehicle.apply_control(control)

  def _update_spectator(self):
    """Position CARLA's visible spectator camera, per CARLA_SPECTATOR_MODE."""
    if self.spectator_mode == "orbit":
      self._orbit_spectator()
    elif self.spectator_mode != "free":
      self._chase_spectator()
    if self.debug_markers:
      self._draw_debug_markers()

  def _chase_spectator(self):
    transform = self.vehicle.get_transform()
    forward = transform.get_forward_vector()
    location = self.carla.Location(
      x=transform.location.x - 8.0 * forward.x,
      y=transform.location.y - 8.0 * forward.y,
      z=transform.location.z + 3.5,
    )
    rotation = self.carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw)
    self.world.get_spectator().set_transform(self.carla.Transform(location, rotation))

  ORBIT_UPDATE_TICKS = 6  # ~3-4 Hz at the ~20 Hz this is called from tick()

  def _orbit_spectator(self, distance=8.0):
    """Recenter on the car periodically, orbiting on whatever direction the
    mouse last dragged the spectator to face.

    CARLA only exposes camera control as set_transform(); free-look drag is
    handled client-side and isn't visible to this script except by reading it
    back. Calling that every tick (~20 Hz) fights the drag almost as fast as
    it can register -- CARLA's own view of "current transform" barely has a
    chance to reflect the mouse before this overwrites it again, so rotating
    looked like it snapped back to wherever the last write put it. Only
    reading/writing every ORBIT_UPDATE_TICKS gives the drag room to land.

    First call ever has no meaningful transform to read back -- CARLA's
    spectator default is an arbitrary high top-down view left over from map
    load, not something worth orbiting off of -- so seed a side view instead.
    """
    if not hasattr(self, "_orbit_tick"):
      self._orbit_tick = 0
      car = self.vehicle.get_transform()
      right = car.get_right_vector()
      seed_loc = self.carla.Location(
        x=car.location.x - distance * right.x,
        y=car.location.y - distance * right.y,
        z=car.location.z + 2.0,
      )
      self.world.get_spectator().set_transform(self.carla.Transform(seed_loc, self.carla.Rotation()))

    self._orbit_tick += 1
    if self._orbit_tick % self.ORBIT_UPDATE_TICKS != 0:
      return

    spectator = self.world.get_spectator()
    forward = spectator.get_transform().get_forward_vector()
    car_loc = self.vehicle.get_transform().location
    new_loc = self.carla.Location(
      x=car_loc.x - distance * forward.x,
      y=car_loc.y - distance * forward.y,
      z=car_loc.z - distance * forward.z + 1.0,
    )
    dx, dy, dz = car_loc.x - new_loc.x, car_loc.y - new_loc.y, car_loc.z - new_loc.z
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    spectator.set_transform(self.carla.Transform(new_loc, self.carla.Rotation(pitch=pitch, yaw=yaw)))

  def tick(self):
    try:
      self.current_frame = self.world.tick()
      self._update_spectator()
    except RuntimeError as exc:
      self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, str(exc)))
      self.exit_event.set()

  def read_state(self):
    pass

  def read_sensors(self, state: SimulatorState):
    velocity = self.vehicle.get_velocity()
    state.velocity = vec3(float(velocity.x), float(velocity.y), float(velocity.z))
    # Report the angle the front wheels actually reached rather than echoing the
    # command back. The echo made carState.steeringAngleDeg a perfect mirror of
    # the request, so any shortfall in the simulated rack was invisible to
    # openpilot and to the paramsd/torqued estimators feeding off it.
    fl = self.vehicle.get_wheel_steer_angle(self.carla.VehicleWheelLocation.FL_Wheel)
    fr = self.vehicle.get_wheel_steer_angle(self.carla.VehicleWheelLocation.FR_Wheel)
    state.steering_angle = -0.5 * (float(fl) + float(fr)) * self.steer_ratio

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
      # CARLA reports accel/gyro in vehicle body frame (UE left-handed,
      # x-forward, y-right, z-up; IMU has zero relative rotation to vehicle,
      # see _sensor_transform()). locationd.py applies meas=[-v2,-v1,-v0] to
      # every raw sample to land in openpilot's device frame (x-forward,
      # y-right, z-down, right-handed). That remap is self-inverse, so we
      # pre-apply it to the true device-frame reading:
      #   device accel = (ax, ay, -az)   -- polar vector, z flips
      #   device gyro  = (-gx, -gy, gz)  -- pseudovector, x/y flip, z(yaw) invariant
      state.imu.accelerometer = vec3(float(acc.z), float(-acc.y), float(-acc.x))
      state.imu.gyroscope = vec3(float(-gyro.z), float(gyro.y), float(gyro.x))
      state.imu.bearing = math.degrees(float(self.latest_imu.compass)) % 360.0
    state.valid = True

  @staticmethod
  def _rgb(image):
    bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    return np.ascontiguousarray(bgra[:, :, 2::-1])

  def _camera_for_frame(self, name):
    deadline = time.monotonic() + self.CAMERA_TIMEOUT
    latest = None
    while time.monotonic() < deadline:
      try:
        image = self.camera_queues[name].get(timeout=max(deadline - time.monotonic(), 0.01))
      except queue.Empty:
        break
      latest = image
      if self.current_frame is None or image.frame >= self.current_frame:
        return image
    if latest is None:
      raise TimeoutError(f"timed out waiting for {name} camera frame {self.current_frame}")
    return latest

  def read_cameras(self):
    try:
      road = self._camera_for_frame("road")
      self.road_bgra_image = np.frombuffer(road.raw_data, dtype=np.uint8).reshape((road.height, road.width, 4))
      if self.dual_camera:
        wide = self._camera_for_frame("wide")
        self.wide_road_bgra_image = np.frombuffer(wide.raw_data, dtype=np.uint8).reshape((wide.height, wide.width, 4))
      self.image_lock.release()
    except (TimeoutError, ValueError) as exc:
      self.status_q.put(QueueMessage(QueueMessageType.TERMINATION_INFO, str(exc)))
      self.exit_event.set()

  def reset(self):
    self.vehicle.apply_control(self.carla.VehicleControl())
    self.vehicle.set_target_velocity(self.carla.Vector3D())
    self.vehicle.set_target_angular_velocity(self.carla.Vector3D())
    self.vehicle.set_transform(self.spawn_transform)

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
