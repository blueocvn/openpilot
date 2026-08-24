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
  # CARLA's Tesla accelerates at roughly 6 m/s² at full throttle. Closed-loop
  # measurements put full braking near 10 m/s²; the old value of 50 mapped an
  # openpilot -3.5 m/s² request to only 7% brake and caused rear impacts.
  THROTTLE_ACCEL = 6.0
  BRAKE_DECEL = 10.0
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
  # The model's lead distance is measured from the camera reference rather than
  # bumper-to-bumper. In the CARLA geometry, contact occurs at a model-reported
  # distance of roughly 5.4 m, so reserve 6 m and one second of reaction travel.
  # This guard consumes only radard's camera-derived lead (radar=False); no
  # CARLA actor position, velocity, or other simulator ground truth is used.
  vision_aeb_probability = 0.8
  vision_aeb_distance = 18.0
  vision_aeb_standstill_buffer = 6.0
  vision_aeb_reaction_time = 1.0
  vision_aeb_max_decel = 3.5

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
    self.vision_aeb_enabled = os.environ.get("CARLA_VISION_AEB", "1") != "0"
    self.vision_aeb_active = False

  @classmethod
  def vision_aeb_accel(cls, requested_accel, v_ego, lead):
    if (not lead.present or lead.radar or lead.modelProb < cls.vision_aeb_probability or
        lead.dRel >= cls.vision_aeb_distance):
      return requested_accel

    available_distance = (lead.dRel - cls.vision_aeb_standstill_buffer -
                          cls.vision_aeb_reaction_time * v_ego)
    if available_distance <= 0.0:
      safe_accel = -cls.vision_aeb_max_decel
    else:
      safe_accel = -min(v_ego ** 2 / (2.0 * available_distance), cls.vision_aeb_max_decel)
    return min(requested_accel, safe_accel)

  def limit_longitudinal_accel(self, requested_accel):
    if not self.vision_aeb_enabled:
      return requested_accel

    lead = self.simulated_car.sm['radarState'].leadOne
    limited_accel = self.vision_aeb_accel(requested_accel, self.simulator_state.speed, lead)
    active = limited_accel < requested_accel
    if active and not self.vision_aeb_active:
      print(f"CARLA vision AEB: dRel={lead.dRel:.2f} m vLead={lead.vLead:.2f} m/s "
            f"prob={lead.modelProb:.2f} accel={requested_accel:.2f}->{limited_accel:.2f} m/s^2")
    elif not active and self.vision_aeb_active:
      print("CARLA vision AEB released")
    self.vision_aeb_active = active
    return limited_accel

  def spawn_world(self, queue: Queue):
    return CarlaWorld(queue, self.host, self.port, self.town, self.spawn_point,
                      self.dual_camera, self.high_quality)
