import math
import os
from multiprocessing import Queue

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.carla.carla_world import CarlaWorld, wsl_host
from openpilot.tools.sim.lib.carla_simulated_car import CarlaSimulatedCar


def scene_cruise_speed(scene: str, *, stock_speed_mps: float) -> float:
  return 30.0 / 3.6 if scene == "motorcycle_weave" else stock_speed_mps


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
               openpilot_longitudinal=False, carla_scene="none", carla_scene_case="alternating",
               carla_scene_seed=42, carla_scene_duration=45.0, carla_report_dir=None,
               experimental_mode=None):
    self.alpha_longitudinal_enabled = openpilot_longitudinal
    self.experimental_mode_enabled = False if carla_scene != "none" and experimental_mode is None else experimental_mode
    super().__init__(dual_camera, high_quality)
    self.simulator_state.cruise_speed = scene_cruise_speed(carla_scene, stock_speed_mps=self.stock_cruise_speed)
    self.host = host or os.environ.get("CARLA_HOST") or wsl_host()
    self.port = port
    self.town = town
    self.spawn_point = spawn_point
    self.test_duration = test_duration
    self.test_run = test_run
    self.carla_scene = carla_scene
    self.carla_scene_case = carla_scene_case
    self.carla_scene_seed = carla_scene_seed
    self.carla_scene_duration = carla_scene_duration
    self.carla_report_dir = carla_report_dir

  def spawn_world(self, queue: Queue):
    return CarlaWorld(queue, self.host, self.port, self.town, self.spawn_point,
                      self.dual_camera, self.high_quality, self.carla_scene, self.carla_scene_case,
                      self.carla_scene_seed, self.carla_scene_duration, self.carla_report_dir)
