from tools.sim.bridge.common import SimulatorBridge
from tools.sim.bridge.carla_world import CarlaWorld


class CarlaBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5
  # Starting point from openpilot/tools/sim/bridge/carla/carla_bridge.py (Model 3);
  # the PI loop around carState.aEgo absorbs the CARLA 0.9.12 physics mismatch.
  THROTTLE_ACCEL = 6.0
  BRAKE_DECEL = 50.0
  longitudinal_accel_kp = 0.8
  longitudinal_accel_ki = 0.35
  longitudinal_accel_integral_limit = 3.0

  def __init__(self, host="127.0.0.1", port=2000, town="Town04_Opt", spawn_point=16,
               vehicle="vehicle.tesla.model3", low_quality=False, weather="ClearNoon",
               auto_engage=True, test_run=False):
    super().__init__(auto_engage=auto_engage, test_run=test_run)
    self.host = host
    self.port = port
    self.town = town
    self.spawn_point = spawn_point
    self.vehicle = vehicle
    self.low_quality = low_quality
    self.weather = weather

  def spawn_world(self, q):
    return CarlaWorld(q, self.host, self.port, self.town, self.spawn_point, vehicle=self.vehicle,
                      low_quality=self.low_quality, weather=self.weather)
