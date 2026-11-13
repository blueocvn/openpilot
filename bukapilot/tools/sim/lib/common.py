"""Simulator-agnostic state shared by the bridge, the simulated car and the
simulated sensors. Ported from openpilot/tools/sim/lib/common.py, kept
Python 3.8 compatible for bukapilot (no PEP 604 unions)."""
import math
import multiprocessing
from abc import ABC, abstractmethod
from collections import namedtuple

import numpy as np

# bukapilot (0.8.13) road camera resolution, same as the legacy tools/sim bridge.
W, H = 1928, 1208

vec3 = namedtuple("vec3", ["x", "y", "z"])


class GPSState:
  def __init__(self):
    self.latitude = 0.
    self.longitude = 0.
    self.altitude = 0.

  def from_xy(self, xy):
    """Fake a lat/lon from a planar xy coordinate when no GNSS fix is available."""
    BASE_LAT = 32.75308505188913
    BASE_LON = -117.2095393365393
    DEG_TO_METERS = 100000

    self.latitude = float(BASE_LAT + xy[0] / DEG_TO_METERS)
    self.longitude = float(BASE_LON + xy[1] / DEG_TO_METERS)
    self.altitude = 0.


class IMUState:
  def __init__(self):
    self.accelerometer = vec3(0., 0., 0.)
    self.gyroscope = vec3(0., 0., 0.)
    self.bearing = 0.


class SimulatorState:
  def __init__(self):
    self.valid = False
    self.is_engaged = False
    self.ignition = True

    self.velocity = vec3(0., 0., 0.)
    self.bearing = 0.
    self.gps = GPSState()
    self.imu = IMUState()

    self.steering_angle = 0.

    self.user_gas = 0.
    self.user_brake = 0.
    self.user_torque = 0.

    self.cruise_button = 0
    self.left_blinker = False
    self.right_blinker = False

  @property
  def speed(self):
    return math.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2 + self.velocity.z ** 2)


class World(ABC):
  def __init__(self):
    self.image_lock = multiprocessing.Semaphore(value=0)
    # CARLA hands back BGRA frames; the camera thread converts them to I420.
    self.road_bgra_image = np.zeros((H, W, 4), dtype=np.uint8)

    self.exit_event = multiprocessing.Event()
    self._reset_event = multiprocessing.Event()

  def notify_reset(self):
    self._reset_event.set()

  def consume_reset_event(self):
    was_reset = self._reset_event.is_set()
    if was_reset:
      self._reset_event.clear()
    return was_reset

  def truth_lead(self):
    """(bumper gap m, lead speed m/s) of the nearest object ahead, for logging only; None when unknown."""
    return None

  def consume_collisions(self):
    """[(other actor type, impulse)] of the ego car since the last call, for logging only."""
    return []

  @abstractmethod
  def apply_controls(self, steer_angle, throttle_out, brake_out):
    pass

  @abstractmethod
  def tick(self):
    pass

  @abstractmethod
  def read_sensors(self, simulator_state: SimulatorState):
    pass

  @abstractmethod
  def read_cameras(self):
    pass

  @abstractmethod
  def close(self, reason: str):
    pass

  @abstractmethod
  def reset(self):
    pass
