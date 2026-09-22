import unittest

from openpilot.tools.sim.exposure_matching import match_exposures


def event(actor_id, direction="right_to_left", gap=7.0, ego=8.0, hold=1.7, **extra):
  return {"actor_id": actor_id, "direction": direction, "entry_gap_m": gap,
          "ego_speed_at_entry_mps": ego, "hold_s": hold, "complete": True,
          "evidence_missing": False, **extra}


class TestExposureMatching(unittest.TestCase):
  def test_deterministic_one_to_one_matching_with_calipers(self):
    result = match_exposures([event(1), event(2, gap=7.8), event(3, direction="left_to_right"), event(4, direction="left_to_right")],
                             [event(10, gap=7.7), event(11, gap=7.1), event(12, direction="left_to_right"), event(13, direction="left_to_right")])
    self.assertEqual([(pair["baseline"]["actor_id"], pair["candidate"]["actor_id"])
                      for pair in result["pairs"]], [(1, 11), (2, 10), (3, 12), (4, 13)])
    self.assertTrue(result["comparison_valid"])

  def test_contact_is_matched_not_discarded(self):
    result = match_exposures([event(1, contact=True), event(2), event(3, direction="left_to_right"), event(4, direction="left_to_right")],
                             [event(10), event(11), event(12, direction="left_to_right"), event(13, direction="left_to_right")])
    self.assertIn(1, [pair["baseline"]["actor_id"] for pair in result["pairs"]])

  def test_hard_unmatched_exposure_invalidates_pair(self):
    base = [event(1), event(2), event(3, direction="left_to_right"), event(4, direction="left_to_right")]
    candidate = [event(10, gap=11), event(11, gap=11), event(12, direction="left_to_right"), event(13, direction="left_to_right")]
    result = match_exposures(base, candidate)
    self.assertFalse(result["comparison_valid"])
    self.assertEqual(result["unmatched_baseline_ids"], [1, 2])
