import math
import os
from multiprocessing import Queue

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.carla.carla_world import CarlaWorld, wsl_host
from openpilot.tools.sim.lib.carla_simulated_car import CarlaSimulatedCar


class CarlaBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5
  simulated_car_class = CarlaSimulatedCar
  stock_cruise_emulation = True
  use_controller_steering_output = True
  # CARLA's Tesla accelerates at roughly 6 m/s² at full throttle and brakes at
  # roughly 50 m/s² at full brake.
  THROTTLE_ACCEL = 6.0
  BRAKE_DECEL = 50.0
  # Keep the simulated Tesla on its stock longitudinal path. The bridge
  # emulates that stock ACC while openpilot controls steering.
  alpha_longitudinal_enabled = False
  openpilot_startup_delay = 3.0
  # Roll away from standstill until the first openpilot engagement. This avoids
  # the stationary model settling on a stop prediction before cruise engages.
  startup_throttle = 0.25
  stock_cruise_speed = 8.0
  longitudinal_accel_kp = 0.8
  longitudinal_accel_ki = 0.35
  longitudinal_accel_integral_limit = 3.0
  longitudinal_overspeed_margin = 0.5

  def __init__(self, dual_camera, high_quality, host=None, port=2000,
               town="Town10HD_Opt", spawn_point=16, test_duration=math.inf, test_run=False,
               openpilot_longitudinal=False):
    self.alpha_longitudinal_enabled = openpilot_longitudinal
    super().__init__(dual_camera, high_quality)
    self.simulator_state.cruise_speed = self.stock_cruise_speed
    self.host = host or os.environ.get("CARLA_HOST") or wsl_host()
    self.port = port
    self.town = town
    self.spawn_point = spawn_point
    self.test_duration = test_duration
    self.test_run = test_run

  def spawn_world(self, queue: Queue):
    return CarlaWorld(queue, self.host, self.port, self.town, self.spawn_point,
                      self.dual_camera, self.high_quality)
