import unittest

from openpilot.selfdrive.controls.lib.vn_traffic_policy import PositiveAccelRamp, TrafficModeConfig


class TestTrafficModeConfig(unittest.TestCase):
  def test_defaults_off_and_rejects_invalid_values(self):
    self.assertEqual(TrafficModeConfig.from_environment({}).mode, False)
    self.assertEqual(TrafficModeConfig.from_environment({}).profile, "balanced")
    for env in ({"VN_TRAFFIC_MODE": "yes"}, {"VN_TRAFFIC_PROFILE": "fast"}):
      with self.assertRaises(ValueError):
        TrafficModeConfig.from_environment(env)

  def test_simulation_gate(self):
    config = TrafficModeConfig.from_environment({"VN_TRAFFIC_MODE": "1", "VN_TRAFFIC_PROFILE": "gentle"})
    self.assertFalse(config.enabled_for(simulation=False, longitudinal_active=True, experimental=False))
    self.assertFalse(config.enabled_for(simulation=True, longitudinal_active=False, experimental=False))
    self.assertFalse(config.enabled_for(simulation=True, longitudinal_active=True, experimental=True))
    self.assertTrue(config.enabled_for(simulation=True, longitudinal_active=True, experimental=False))


class TestPositiveAccelRamp(unittest.TestCase):
  def test_positive_acceleration_ramps_and_caps_without_raising_request(self):
    ramp = PositiveAccelRamp("balanced", dt=0.05)
    self.assertAlmostEqual(ramp.apply(1.5, speed_mps=8.0, active=True), 0.04)
    self.assertAlmostEqual(ramp.apply(1.5, speed_mps=8.0, active=True), 0.08)
    for _ in range(30):
      result = ramp.apply(1.5, speed_mps=8.0, active=True)
    self.assertAlmostEqual(result, 1.0)
    self.assertAlmostEqual(ramp.apply(0.3, speed_mps=8.0, active=True), 0.3)

  def test_deceleration_is_untouched_and_resets_ramp(self):
    ramp = PositiveAccelRamp("gentle", dt=0.1)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=8.0, active=True), 0.06)
    self.assertEqual(ramp.apply(-2.0, speed_mps=8.0, active=True), -2.0)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=8.0, active=True), 0.06)
    self.assertEqual(ramp.apply(0.0, speed_mps=8.0, active=True), 0.0)

  def test_disengage_and_standstill_reset_without_modifying_stock_output(self):
    ramp = PositiveAccelRamp("responsive", dt=0.1)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=8.0, active=True), 0.1)
    self.assertEqual(ramp.apply(1.0, speed_mps=8.0, active=False), 1.0)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=8.0, active=True), 0.1)
    self.assertEqual(ramp.apply(1.0, speed_mps=0.0, active=True, standstill=True), 1.0)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=8.0, active=True), 0.1)

  def test_speed_fade_and_invalid_input_preserve_stock(self):
    ramp = PositiveAccelRamp("balanced", dt=0.1)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=40 / 3.6, active=True), 0.08)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=45 / 3.6, active=True), 0.58)
    self.assertEqual(ramp.apply(1.0, speed_mps=50 / 3.6, active=True), 1.0)
    self.assertEqual(ramp.apply(0.9, speed_mps=8.0, active=True, input_valid=False), 0.9)
    self.assertAlmostEqual(ramp.apply(1.0, speed_mps=8.0, active=True), 0.08)

  def test_identical_observed_prefix_has_identical_outputs(self):
    prefix = [(0.5, 8.0), (1.2, 8.1), (-0.5, 8.0), (1.0, 7.9)]
    outputs = []
    for unseen_future in ([(2.0, 9.0)], [(-3.0, 7.0)]):
      ramp = PositiveAccelRamp("balanced", dt=0.1)
      outputs.append([ramp.apply(a, speed_mps=v, active=True) for a, v in prefix + unseen_future][:len(prefix)])
    self.assertEqual(outputs[0], outputs[1])


if __name__ == "__main__":
  unittest.main()
