import time

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging

from openpilot.common.realtime import DT_DMON
from openpilot.tools.sim.lib.camerad import Camerad

from typing import TYPE_CHECKING
if TYPE_CHECKING:
  from openpilot.tools.sim.lib.common import World, SimulatorState


class SimulatedSensors:
  """Simulates the C3 sensors (acc, gyro, gps, peripherals, dm state, cameras) to OpenPilot"""

  def __init__(self, dual_camera=False):
    self.pm = messaging.PubMaster(['accelerometer', 'gyroscope', 'gpsLocationExternal', 'driverStateV2', 'driverMonitoringState', 'peripheralState'])
    self.camerad = Camerad(dual_camera=dual_camera)
    self.last_perp_update = 0
    self.last_dmon_update = 0
    self.camera_stats_start = time.monotonic()
    self.camera_stats_frames = 0
    self.camera_stats_conversion_s = 0.0

  def send_imu_message(self, simulator_state: 'SimulatorState'):
    for _ in range(5):
      dat = messaging.new_message('accelerometer', valid=True)
      dat.accelerometer.timestamp = dat.logMonoTime  # TODO: use the IMU timestamp
      dat.accelerometer.init('acceleration')
      dat.accelerometer.acceleration.v = [simulator_state.imu.accelerometer.x, simulator_state.imu.accelerometer.y, simulator_state.imu.accelerometer.z]
      self.pm.send('accelerometer', dat)

      dat = messaging.new_message('gyroscope', valid=True)
      dat.gyroscope.timestamp = dat.logMonoTime  # TODO: use the IMU timestamp
      dat.gyroscope.init('gyroUncalibrated')
      dat.gyroscope.gyroUncalibrated.v = [simulator_state.imu.gyroscope.x, simulator_state.imu.gyroscope.y, simulator_state.imu.gyroscope.z]
      self.pm.send('gyroscope', dat)

  def send_gps_message(self, simulator_state: 'SimulatorState'):
    if not simulator_state.valid:
      return

    # transform from vel to NED
    velNED = [
      -simulator_state.velocity.y,
      simulator_state.velocity.x,
      simulator_state.velocity.z,
    ]

    for _ in range(10):
      dat = messaging.new_message('gpsLocationExternal', valid=True)
      dat.gpsLocationExternal = {
        "unixTimestampMillis": int(time.time() * 1000),  # noqa: TID251
        "flags": 1,  # valid fix
        "horizontalAccuracy": 1.0,
        "verticalAccuracy": 1.0,
        "speedAccuracy": 0.1,
        "bearingAccuracyDeg": 0.1,
        "vNED": velNED,
        "bearingDeg": simulator_state.imu.bearing,
        "latitude": simulator_state.gps.latitude,
        "longitude": simulator_state.gps.longitude,
        "altitude": simulator_state.gps.altitude,
        "speed": simulator_state.speed,
        "source": log.GpsLocationData.SensorSource.ublox,
      }

      self.pm.send('gpsLocationExternal', dat)

  def send_peripheral_state(self):
    dat = messaging.new_message('peripheralState')
    dat.valid = True
    dat.peripheralState = {
      'pandaType': log.PandaState.PandaType.blackPanda,
      'voltage': 12000,
      'current': 5678,
      'fanSpeedRpm': 1000
    }
    self.pm.send('peripheralState', dat)

  def send_fake_driver_monitoring(self):
    # dmonitoringmodeld output
    dat = messaging.new_message('driverStateV2')
    dat.driverStateV2.leftDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.leftDriverData.faceProb = 1.0
    dat.driverStateV2.rightDriverData.faceOrientation = [0., 0., 0.]
    dat.driverStateV2.rightDriverData.faceProb = 1.0
    self.pm.send('driverStateV2', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState', valid=True)
    dm = dat.driverMonitoringState
    dm.alertLevel = log.DriverMonitoringState.AlertLevel.none
    dm.activePolicy = log.DriverMonitoringState.MonitoringPolicy.vision
    dm.visionPolicyState.faceDetected = True
    dm.visionPolicyState.isDistracted = False
    dm.visionPolicyState.awarenessPercent = 100
    self.pm.send('driverMonitoringState', dat)

  def send_camera_images(self, world: 'World'):
    world.image_lock.acquire()
    conversion_start = time.perf_counter()
    road_yuv = (self.camerad.bgra_to_yuv(world.road_bgra_image)
                if world.road_bgra_image is not None else self.camerad.rgb_to_yuv(world.road_image))
    wide_yuv = None
    if world.dual_camera:
      wide_yuv = (self.camerad.bgra_to_yuv(world.wide_road_bgra_image)
                  if world.wide_road_bgra_image is not None else self.camerad.rgb_to_yuv(world.wide_road_image))

    # RGB->NV12 conversion is CPU-heavy. Timestamp both camera frames only
    # after conversion so modeld receives a synchronized stereo pair.
    timestamp_ns = time.monotonic_ns()
    self.camerad.cam_send_yuv_road(road_yuv, timestamp_ns)
    if wide_yuv is not None:
      self.camerad.cam_send_yuv_wide_road(wide_yuv, timestamp_ns)

    self.camera_stats_frames += 1
    self.camera_stats_conversion_s += time.perf_counter() - conversion_start
    now = time.monotonic()
    elapsed = now - self.camera_stats_start
    if elapsed >= 2.0:
      camera_count = 2 if wide_yuv is not None else 1
      stats = f"SIM camera: {self.camera_stats_frames / elapsed:.1f} FPS "
      stats += f"({self.camera_stats_conversion_s * 1000 / self.camera_stats_frames:.2f} ms RGB→NV12, "
      stats += f"{camera_count} camera{'s' if camera_count > 1 else ''})"
      print(stats)
      self.camera_stats_start = now
      self.camera_stats_frames = 0
      self.camera_stats_conversion_s = 0.0

  def update(self, simulator_state: 'SimulatorState', world: 'World'):
    now = time.monotonic()
    self.send_imu_message(simulator_state)
    self.send_gps_message(simulator_state)

    if (now - self.last_dmon_update) > DT_DMON/2:
      self.send_fake_driver_monitoring()
      self.last_dmon_update = now

    # SERVICE_LIST wants peripheralState at 2 Hz. SubMaster fails freq_ok for
    # publishing too fast as well as too slow, so this interval is 1/2 Hz.
    if (now - self.last_perp_update) > 0.5:
      self.send_peripheral_state()
      self.last_perp_update = now
