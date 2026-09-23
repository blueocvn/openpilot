import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc


def cruise_messages():
  car_state = messaging.new_message("carState").carState
  car_state.vEgo = 8.0
  car_state.vCruise = 30.0
  car_state.aEgo = 0.0
  car_state.standstill = False
  car_state.steeringAngleDeg = 0.0
  controls_state = messaging.new_message("controlsState").controlsState
  controls_state.longControlState = LongCtrlState.pid
  selfdrive_state = messaging.new_message("selfdriveState").selfdriveState
  selfdrive_state.experimentalMode = False
  car_control = messaging.new_message("carControl").carControl
  car_control.orientationNED = [0.0, 0.0, 0.0]
  model = messaging.new_message("modelV2").modelV2
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6
  return {
    "carState": car_state,
    "controlsState": controls_state,
    "selfdriveState": selfdrive_state,
    "carControl": car_control,
    "vehicleParameters": messaging.new_message("vehicleParameters").vehicleParameters,
    "radarState": messaging.new_message("radarState").radarState,
    "modelV2": model,
  }


def closing_lead_messages():
  messages = cruise_messages()
  lead = messages["radarState"].leadOne
  lead.present = True
  lead.dRel = 10.0
  lead.vLead = 5.0
  lead.vRel = -3.0
  return messages


class TestTrafficPlannerIntegration(unittest.TestCase):
  def test_opt_in_only_reduces_positive_acceleration_in_simulation(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                         steerRatio=12.0, wheelbase=2.9)
    env = {"SIMULATION": "1", "VN_TRAFFIC_MODE": "0", "VN_TRAFFIC_PROFILE": "balanced"}
    with patch.dict(os.environ, env):
      stock = LongitudinalPlanner(cp)
    env["VN_TRAFFIC_MODE"] = "1"
    with patch.dict(os.environ, env):
      traffic = LongitudinalPlanner(cp)
    stock.update(cruise_messages())
    traffic.update(cruise_messages())
    self.assertAlmostEqual(stock.output_a_target, 0.064)
    self.assertLess(traffic.output_a_target, stock.output_a_target)
    self.assertAlmostEqual(traffic.output_a_target, 0.8 * traffic.dt)
    self.assertEqual(traffic.output_should_stop, stock.output_should_stop)

  def test_invalid_configuration_fails_at_startup(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True)
    with patch.dict(os.environ, {"VN_TRAFFIC_MODE": "1", "VN_TRAFFIC_PROFILE": "not-a-profile"}):
      with self.assertRaises(ValueError):
        LongitudinalPlanner(cp)

  def test_mpc_rejects_headway_override_that_shortens_stock_or_is_invalid(self):
    mpc = LongitudinalMpc.__new__(LongitudinalMpc)
    radar = messaging.new_message("radarState").radarState
    for override in (1.0, float("nan"), float("inf")):
      with self.assertRaises(ValueError):
        mpc.update(radar, t_follow_override=override)

  def test_mode_one_increases_mpc_headway_for_observed_closing_lead(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                         steerRatio=12.0, wheelbase=2.9)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "0"}):
      stock = LongitudinalPlanner(cp)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "1"}):
      traffic = LongitudinalPlanner(cp)
    stock.update(closing_lead_messages())
    traffic.update(closing_lead_messages())
    self.assertAlmostEqual(stock.mpc.params[0, 4], 1.25)
    self.assertGreater(traffic.mpc.params[0, 4], stock.mpc.params[0, 4])
    self.assertEqual(traffic.traffic_follow.diagnostic["stage"], "closing_lead")


if __name__ == "__main__":
  unittest.main()
