from itertools import pairwise
from types import SimpleNamespace
import unittest

from openpilot.selfdrive.controls.lib.vn_traffic_policy import PositiveAccelRamp, TrafficFollowPolicy, TrafficModeConfig


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


class TestTrafficFollowPolicy(unittest.TestCase):
  @staticmethod
  def lead(*, present=True, distance=12.0, speed=5.0):
    return SimpleNamespace(present=present, dRel=distance, vLead=speed)

  def test_closing_lead_adds_bounded_headway_and_uses_either_lead(self):
    policy = TrafficFollowPolicy(dt=0.05)
    absent = self.lead(present=False)
    lead_two = self.lead(distance=10.0, speed=5.0)
    outputs = [policy.apply(1.45, speed_mps=8.0, lead_one=absent, lead_two=lead_two,
                            active=True, radar_valid=True) for _ in range(20)]
    self.assertGreater(outputs[-1], 1.45)
    self.assertLessEqual(outputs[-1], 1.75)
    self.assertTrue(all(a <= b for a, b in pairwise(outputs)))

  def test_never_shortens_stock_and_caps_relaxed_personality(self):
    policy = TrafficFollowPolicy(dt=0.05)
    lead = self.lead(distance=8.0, speed=4.0)
    for _ in range(30):
      result = policy.apply(1.75, speed_mps=8.0, lead_one=lead, lead_two=lead,
                            active=True, radar_valid=True)
    self.assertGreaterEqual(result, 1.75)
    self.assertLessEqual(result, 2.05)

  def test_non_closing_distant_and_high_speed_leads_keep_stock(self):
    cases = [
      (8.0, self.lead(distance=10.0, speed=7.6)),
      (8.0, self.lead(distance=50.0, speed=5.0)),
      (50 / 3.6, self.lead(distance=8.0, speed=4.0)),
    ]
    for speed, lead in cases:
      policy = TrafficFollowPolicy(dt=0.05)
      self.assertEqual(policy.apply(1.45, speed_mps=speed, lead_one=lead, lead_two=lead,
                                    active=True, radar_valid=True), 1.45)

  def test_invalid_or_inactive_input_resets_to_stock(self):
    policy = TrafficFollowPolicy(dt=0.05)
    lead = self.lead(distance=8.0, speed=4.0)
    raised = policy.apply(1.45, speed_mps=8.0, lead_one=lead, lead_two=lead,
                          active=True, radar_valid=True)
    self.assertGreater(raised, 1.45)
    self.assertEqual(policy.apply(1.45, speed_mps=8.0, lead_one=lead, lead_two=lead,
                                  active=True, radar_valid=False), 1.45)
    self.assertEqual(policy.apply(1.45, speed_mps=8.0, lead_one=lead, lead_two=lead,
                                  active=False, radar_valid=True), 1.45)

  def test_headway_falls_at_bounded_rate_when_threat_clears(self):
    policy = TrafficFollowPolicy(dt=0.05)
    lead = self.lead(distance=8.0, speed=4.0)
    for _ in range(20):
      raised = policy.apply(1.45, speed_mps=8.0, lead_one=lead, lead_two=lead,
                            active=True, radar_valid=True)
    cleared = policy.apply(1.45, speed_mps=8.0, lead_one=self.lead(present=False),
                           lead_two=self.lead(present=False), active=True, radar_valid=True)
    self.assertGreater(cleared, 1.45)
    self.assertLess(cleared, raised)


if __name__ == "__main__":
  unittest.main()
