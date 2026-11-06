"""Simulated EON/C2 sensors for bukapilot: IMU, GPS, peripheral and driver
monitoring state, and the road camera.

The IMU differs from the legacy bridge on purpose: locationd (0.8.13) only
consumes sensorEvents with sensor=SENSOR_ACCELEROMETER(1)/type=1 and
sensor=SENSOR_GYRO_UNCALIBRATED(5)/type=16, and drops any reading whose
timestamp is more than 100 ms from the log time. The legacy bridge sent
sensor=4 with no timestamp, so locationd never used its IMU at all.
"""
import time

import cereal.messaging as messaging
from cereal import log
from common.realtime import DT_DMON
from tools.sim_carla.lib.camerad import Camerad

SENSOR_ACCELEROMETER = 1
SENSOR_GYRO_UNCALIBRATED = 5
SENSOR_TYPE_ACCELEROMETER = 1
SENSOR_TYPE_GYROSCOPE_UNCALIBRATED = 16


class SimulatedSensors:
  def __init__(self):
    self.pm = messaging.PubMaster(['sensorEvents', 'gpsLocationExternal', 'driverState',
                                   'driverMonitoringState', 'peripheralState'])
    self.camerad = Camerad()
    self.idx = 0
    self.last_perp_update = 0.
    self.last_dmon_update = 0.
    self.camera_stats_start = time.monotonic()
    self.camera_stats_frames = 0
    self.camera_stats_conversion_s = 0.

  def send_imu_message(self, s):
    dat = messaging.new_message('sensorEvents', 2)
    accel, gyro = dat.sensorEvents[0], dat.sensorEvents[1]

    accel.sensor = SENSOR_ACCELEROMETER
    accel.type = SENSOR_TYPE_ACCELEROMETER
    accel.timestamp = dat.logMonoTime
    accel.init('acceleration')
    accel.acceleration.v = [s.imu.accelerometer.x, s.imu.accelerometer.y, s.imu.accelerometer.z]

    gyro.sensor = SENSOR_GYRO_UNCALIBRATED
    gyro.type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
    gyro.timestamp = dat.logMonoTime
    gyro.init('gyroUncalibrated')
    gyro.gyroUncalibrated.v = [s.imu.gyroscope.x, s.imu.gyroscope.y, s.imu.gyroscope.z]

    self.pm.send('sensorEvents', dat)

  def send_gps_message(self, s):
    if not s.valid:
      return

    # CARLA (x east, y south) velocity -> NED
    vel_ned = [-s.velocity.y, s.velocity.x, s.velocity.z]

    dat = messaging.new_message('gpsLocationExternal')
    dat.gpsLocationExternal = {
      "timestamp": int(time.time() * 1000),
      "flags": 1,  # valid fix
      "accuracy": 1.0,
      "verticalAccuracy": 1.0,
      "speedAccuracy": 0.1,
      "bearingAccuracyDeg": 0.1,
      "vNED": vel_ned,
      "bearingDeg": s.imu.bearing,
      "latitude": s.gps.latitude,
      "longitude": s.gps.longitude,
      "altitude": s.gps.altitude,
      "speed": s.speed,
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
      'fanSpeedRpm': 1000,
    }
    self.pm.send('peripheralState', dat)

  def send_fake_driver_monitoring(self):
    # dmonitoringmodeld output (the real one is disabled on PC)
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    self.pm.send('driverState', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    self.pm.send('driverMonitoringState', dat)

  def send_camera_images(self, world):
    world.image_lock.acquire()
    start = time.perf_counter()
    self.camerad.send_bgra(world.road_bgra_image)

    self.camera_stats_frames += 1
    self.camera_stats_conversion_s += time.perf_counter() - start
    now = time.monotonic()
    elapsed = now - self.camera_stats_start
    if elapsed >= 10.0:
      print(f"SIM camera: {self.camera_stats_frames / elapsed:.1f} FPS "
            f"({self.camera_stats_conversion_s * 1000 / self.camera_stats_frames:.1f} ms BGRA->I420)")
      self.camera_stats_start = now
      self.camera_stats_frames = 0
      self.camera_stats_conversion_s = 0.

  def update(self, s):
    """Called from the 100 Hz bridge loop."""
    now = time.monotonic()
    self.send_imu_message(s)
    if self.idx % 10 == 0:  # GPS at 10 Hz
      self.send_gps_message(s)

    if (now - self.last_dmon_update) > DT_DMON / 2:
      self.send_fake_driver_monitoring()
      self.last_dmon_update = now

    if (now - self.last_perp_update) > 0.25:
      self.send_peripheral_state()
      self.last_perp_update = now
    self.idx += 1
