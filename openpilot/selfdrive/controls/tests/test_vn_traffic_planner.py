import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS
from openpilot.tools.sim.lib.carla_radar import encode_radar
import numpy as np


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


def cutin_messages():
  messages = cruise_messages()
  path_x = [float(x) for x in range(0, 61, 2)]
  messages["modelV2"].position.t = [x / 8.33 for x in path_x]
  messages["modelV2"].position.x = path_x
  messages["modelV2"].position.y = [0.] * len(path_x)
  messages["modelV2"].leadsV3 = [{"prob": 0.8, "probTime": 0.,
                                   "t": [0., 2., 4., 6., 8., 10.],
                                   "x": [20., 28., 36., 44., 52., 60.],
                                   "y": [3., 0.5, -1., -2., -2., -2.],
                                   "v": [4.] * 6, "xStd": [2.] * 6,
                                   "yStd": [0.3] * 6}]
  return messages


class ModelMaster(dict):
  def __init__(self, messages):
    super().__init__(messages)
    self.updated = dict.fromkeys(messages, True)
    self.logMonoTime = dict.fromkeys(messages, 1_000_000_000)
    self.tick_time_ns = 1_000_000_000

  def advance(self):
    self.tick_time_ns += 50_000_000
    self.logMonoTime["modelV2"] = self.tick_time_ns

  def all_checks(self, service_list=None):
    return True


class TestTrafficPlannerIntegration(unittest.TestCase):
  def test_opt_in_radar_cutin_adds_future_obstacle_only_to_enhancement(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                         steerRatio=12.0, wheelbase=2.9)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "1", "VN_RADAR_CUTIN": "1"}):
      enhanced = LongitudinalPlanner(cp)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "1", "VN_RADAR_CUTIN": "0"}):
      baseline = LongitudinalPlanner(cp)
    messages = cruise_messages()
    messages["modelV2"].position.x = [float(x) for x in range(51)]
    messages["modelV2"].position.y = [0.0] * 51
    messages["customReservedRawData0"] = b""
    master = ModelMaster(messages)
    for i in range(5):
      if i:
        master.advance()
      x, y = 12.0 - 0.2*i, 3.7 - 0.2*i
      master["customReservedRawData0"] = encode_radar(i, master.tick_time_ns,
        [(x - 0.2, y - 0.15, 0.0, -4.0), (x + 0.2, y + 0.15, 0.1, -3.9)])
      master.logMonoTime["customReservedRawData0"] = master.tick_time_ns
      baseline.update(master)
      enhanced.update(master)
    self.assertEqual(enhanced.radar_cut_in_predictor.diagnostic["stage"], "confirmed")
    self.assertLess(enhanced.mpc.params[np.searchsorted(T_IDXS, 0.5), 2],
                    baseline.mpc.params[np.searchsorted(T_IDXS, 0.5), 2])
    self.assertLess(enhanced.mpc.v_solution[np.searchsorted(T_IDXS, 1.0)],
                    baseline.mpc.v_solution[np.searchsorted(T_IDXS, 1.0)])

  def test_radar_cutin_is_disabled_for_original_experimental_and_non_simulation(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                         steerRatio=12.0, wheelbase=2.9)
    for simulation, mode, experimental in (("1", "0", False), ("1", "1", True), ("0", "1", False)):
      with patch.dict(os.environ, {"SIMULATION": simulation, "VN_TRAFFIC_MODE": mode, "VN_RADAR_CUTIN": "1"}):
        planner = LongitudinalPlanner(cp)
      messages = cruise_messages()
      messages["selfdriveState"].experimentalMode = experimental
      messages["customReservedRawData0"] = encode_radar(1, 1_000_000_000,
        [(12, 3, 0, -4), (12.1, 3.1, 0, -4)])
      master = ModelMaster(messages)
      for _ in range(5):
        planner.update(master)
        master.advance()
      self.assertIsNone(planner.radar_cut_in_predictor.threat)

  def test_mode_one_uses_model_cutin_only_after_confirmation(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                         steerRatio=12.0, wheelbase=2.9)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "0"}):
      stock = LongitudinalPlanner(cp)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "1"}):
      enhanced = LongitudinalPlanner(cp)
    master = ModelMaster(cutin_messages())
    for index in range(3):
      if index:
        master.advance()
      stock.update(master)
      enhanced.update(master)
      if index < 2:
        self.assertNotEqual(enhanced.mpc.source, LongitudinalPlanSource.lead2)
    self.assertEqual(enhanced.cut_in_predictor.diagnostic["stage"], "confirmed")
    self.assertLess(enhanced.mpc.params[np.searchsorted(T_IDXS, 1.2), 2],
                    stock.mpc.params[np.searchsorted(T_IDXS, 1.2), 2])
    self.assertNotEqual(stock.mpc.source, LongitudinalPlanSource.lead2)
    self.assertLess(enhanced.output_a_target, stock.output_a_target)

  def test_experimental_mode_does_not_apply_cutin(self):
    cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                         steerRatio=12.0, wheelbase=2.9)
    with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "1"}):
      planner = LongitudinalPlanner(cp)
    master = ModelMaster(cutin_messages())
    master["selfdriveState"].experimentalMode = True
    for index in range(3):
      if index:
        master.advance()
      planner.update(master)
    self.assertNotEqual(planner.mpc.source, LongitudinalPlanSource.lead2)

  def test_future_cut_in_obstacle_reduces_mpc_speed_before_overlap(self):
    radar = messaging.new_message("radarState").radarState
    baseline = LongitudinalMpc()
    baseline.set_cur_state(8.33, 0.0)
    baseline.update(radar)
    enhanced = LongitudinalMpc()
    enhanced.set_cur_state(8.33, 0.0)
    obstacle = np.where(T_IDXS >= 1.0, 11.0 + 4.0 * T_IDXS, np.inf)
    enhanced.update(radar, cut_in_obstacle=obstacle)
    self.assertTrue(np.isinf(obstacle[0]))
    self.assertEqual(enhanced.source, LongitudinalPlanSource.lead2)
    self.assertLess(enhanced.v_solution[np.searchsorted(T_IDXS, 1.0)],
                    baseline.v_solution[np.searchsorted(T_IDXS, 1.0)])

  def test_closer_stock_lead_is_never_replaced_by_cut_in(self):
    radar = closing_lead_messages()["radarState"]
    mpc = LongitudinalMpc()
    mpc.set_cur_state(8.33, 0.0)
    obstacle = np.where(T_IDXS >= 1.0, 30.0 + 5.0 * T_IDXS, np.inf)
    mpc.update(radar, cut_in_obstacle=obstacle)
    self.assertEqual(mpc.source, LongitudinalPlanSource.lead0)
    self.assertLess(mpc.params[np.searchsorted(T_IDXS, 1.0), 2], 30.0)

  def test_cutin_winning_only_far_last_stage_does_not_claim_early_stock_braking(self):
    radar = closing_lead_messages()["radarState"]
    baseline = LongitudinalMpc()
    baseline.set_cur_state(8.33, 0.0)
    baseline.update(radar)
    obstacle = baseline.params[:, 2] + 100.0
    obstacle[-1] = baseline.params[-1, 2] - 0.1
    mpc = LongitudinalMpc()
    mpc.set_cur_state(8.33, 0.0)
    mpc.update(radar, cut_in_obstacle=obstacle)
    self.assertEqual(mpc.source, LongitudinalPlanSource.lead0)

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
