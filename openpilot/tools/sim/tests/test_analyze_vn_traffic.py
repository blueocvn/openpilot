from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from openpilot.tools.sim.analyze_vn_traffic import compare_traffic_runs, main, summarize_traffic_records


def control(t, speed, request, *, active=True):
  return {"type": "control_sample", "scene_time_s": t, "ego_speed_mps": speed,
          "ego_brake": 0.0, "openpilot": {"long_active": active, "requested_accel_mps2": request}}


class TestTrafficAnalysis(unittest.TestCase):
  def test_streaming_metrics_exclude_comfort_after_contact(self):
    records = [
      control(0.0, 1.0, 1.0), control(0.1, 1.1, 1.0), control(0.2, 1.2, 1.0),
      {"type": "ground_truth", "scene_time_s": 0.2, "ego_speed_mps": 1.2,
       "actors": [{"role": "cut_in", "distance_to_ego_m": 5.0}],
       "openpilot": {"radar_lead_one": {"present": True, "distance_m": 6.0,
                                         "relative_speed_mps": -2.0}}},
      {"type": "collision", "scene_time_s": 0.25, "other_actor_id": 4},
      control(0.3, 0.0, -2.0), control(0.4, 0.0, -2.0),
    ]
    summary = summarize_traffic_records(iter(records))
    self.assertEqual(summary["contact_count"], 1)
    self.assertEqual(summary["comfort_sample_count"], 3)
    self.assertAlmostEqual(summary["p95_abs_actual_jerk_mps3"], 0.0, places=5)
    self.assertAlmostEqual(summary["minimum_observed_ttc_s"], 3.0)
    self.assertAlmostEqual(summary["minimum_actor_clearance_m"], 5.0)
    self.assertAlmostEqual(summary["distance_m"], 0.1 * (1.0 + 1.1) / 2 + 0.1 * (1.1 + 1.2) / 2 +
                           0.1 * (1.2 + 0.0) / 2, places=5)
    self.assertGreater(summary["comfort_excluded_duration_s"], 0)

  def test_missing_control_samples_invalidate_run(self):
    summary = summarize_traffic_records(iter([control(0.0, 2.0, 0.0), control(0.5, 2.0, 0.0)]))
    self.assertFalse(summary["data_valid"])
    self.assertEqual(summary["control_gap_count"], 1)

  def test_direct_carla_acceleration_takes_precedence_over_speed_difference(self):
    samples = [control(0.0, 1.0, 0.0), control(0.1, 1.0, 0.0), control(0.2, 1.0, 0.0)]
    for sample, acceleration in zip(samples, (0.0, 0.5, 1.0), strict=True):
      sample["ego_accel_mps2"] = acceleration
    summary = summarize_traffic_records(iter(samples))
    self.assertAlmostEqual(summary["p95_abs_actual_jerk_mps3"], 5.0)
    self.assertEqual(summary["actual_accel_source"], "carla")

  def test_repeated_collision_callbacks_count_one_actor_contact(self):
    summary = summarize_traffic_records(iter([
      control(0.0, 1.0, 0.0), control(0.1, 1.0, 0.0), control(0.2, 1.0, 0.0),
      {"type": "collision", "scene_time_s": 0.25, "other_actor_id": 7},
      {"type": "collision", "scene_time_s": 0.26, "other_actor_id": 7},
    ]))
    self.assertEqual(summary["contact_count"], 1)
    self.assertEqual(summary["contact_event_count"], 2)

  def test_manifest_duration_does_not_overwrite_measured_duration(self):
    with TemporaryDirectory() as directory:
      report = Path(directory)
      records = [control(0.0, 1.0, 0.0), control(0.1, 1.0, 0.0), control(0.2, 1.0, 0.0)]
      (report / "events.jsonl").write_text("".join(json.dumps(record) + "\n" for record in records))
      (report / "manifest.json").write_text(json.dumps({"duration_s": 60, "vn_traffic_mode": "0"}))
      output = StringIO()
      with patch("sys.argv", ["analyze_vn_traffic.py", str(report)]), redirect_stdout(output):
        main()
      summary = json.loads(output.getvalue())["runs"][0]
      self.assertAlmostEqual(summary["duration_s"], 0.2)
      self.assertEqual(summary["configured_duration_s"], 60)

  def test_pairwise_acceptance_rejects_contact_or_progress_regression(self):
    baseline = {"vn_traffic_mode": "0", "seed": 42,
                "p95_abs_actual_jerk_mps3": 4.0, "accel_decel_switch_count": 3,
                "distance_m": 100.0, "contact_count": 0, "disengagement_count": 0,
                "minimum_observed_ttc_s": 2.0, "minimum_actor_clearance_m": 5.0, "data_valid": True}
    candidate = {**baseline, "vn_traffic_mode": "1", "p95_abs_actual_jerk_mps3": 3.0, "distance_m": 96.0}
    self.assertTrue(compare_traffic_runs(baseline, candidate)["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "contact_count": 1})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "distance_m": 94.0})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "data_valid": False})["pass"])
    self.assertFalse(compare_traffic_runs({**baseline, "actual_accel_source": "carla"},
                                          {**candidate, "actual_accel_source": "speed_difference"})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "vn_traffic_mode": "0"})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "seed": 43})["pass"])


if __name__ == "__main__":
  unittest.main()
