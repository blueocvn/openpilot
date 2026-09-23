from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from openpilot.tools.sim.analyze_vn_traffic import (assess_run, classify_lead, compare_traffic_runs, main,
                                                    runtime_receipt_matches_episode, summarize_planner_stage_records,
                                                    summarize_traffic_records)


def control(t, speed, request, *, active=True):
  return {"type": "control_sample", "scene_time_s": t, "ego_speed_mps": speed,
          "ego_brake": 0.0, "openpilot": {"long_active": active, "requested_accel_mps2": request}}


class TestTrafficAnalysis(unittest.TestCase):
  def test_lead_attribution_distinguishes_missing_other_cut_in_and_ambiguous(self):
    bike = {"id": 7, "role": "cut_in", "ego_forward_gap_m": 7.0, "ego_lateral_offset_m": 0.2}
    other = {"id": 8, "role": "background", "ego_forward_gap_m": 15.0, "ego_lateral_offset_m": -1.0}
    lead = {"present": True, "distance_m": 7.0, "lateral_m": 0.2}
    self.assertEqual(classify_lead({"present": False}, [bike], 7), "no_lead_observed")
    self.assertEqual(classify_lead(lead, [bike, other], 7), "matched_cut_in")
    self.assertEqual(classify_lead({**lead, "distance_m": 15.0, "lateral_m": -1.0},
                                   [bike, other], 7), "matched_other_actor")
    self.assertEqual(classify_lead(lead, [bike, {**bike, "id": 9}], 7), "ambiguous")

  def test_unmatched_observed_lead_is_result_not_missing_actor_telemetry(self):
    bike = {"id": 7, "role": "cut_in", "ego_forward_gap_m": 7.0, "ego_lateral_offset_m": 0.2}
    records = [{"type": "cut_in_started", "scene_time_s": 0.0, "actor_id": 7}]
    for index in range(3):
      sample = control(index / 10, 8.0, 0.0)
      sample["actors"] = [bike]
      sample["host_monotonic_ns"] = 1_000_000_000 + index * 100_000_000
      sample["openpilot"]["snapshot_monotonic_ns"] = sample["host_monotonic_ns"] - 50_000_000
      sample["openpilot"]["radar_lead_one"] = {"present": True, "distance_m": 30.0, "lateral_m": 0.0}
      records.append(sample)
    summary = summarize_traffic_records(iter(records))
    self.assertEqual(summary["lead_attribution_counts"]["ambiguous"], 3)
    self.assertTrue(summary["lead_attribution_valid"])

  def test_stale_openpilot_snapshot_invalidates_lead_attribution(self):
    sample = control(0.0, 8.0, 0.0)
    sample.update({"actors": [], "host_monotonic_ns": 1_000_000_000})
    sample["openpilot"]["snapshot_monotonic_ns"] = 500_000_000
    summary = summarize_traffic_records(iter([
      {"type": "cut_in_started", "scene_time_s": 0.0, "actor_id": 7}, sample,
    ]))
    self.assertFalse(summary["lead_attribution_valid"])

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

  def test_disengaged_run_is_not_valid_longitudinal_evidence(self):
    summary = summarize_traffic_records(iter(control(t / 10, 2.0, 0.0, active=False) for t in range(5)))
    self.assertFalse(summary["data_valid"])

  def test_five_hz_samples_are_not_valid_ten_hz_telemetry(self):
    summary = summarize_traffic_records(iter(control(t / 5, 2.0, 0.0) for t in range(5)))
    self.assertFalse(summary["data_valid"])

  def test_duplicate_timestamp_is_invalid_not_a_division_error(self):
    summary = summarize_traffic_records(iter([
      control(0.0, 1.0, 0.0), control(0.1, 1.1, 0.2), control(0.1, 1.2, 0.4),
    ]))
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

  def test_tracking_error_is_reported_not_a_post_hoc_run_gate(self):
    records = [control(0.0, 1.0, 0.0), control(0.1, 1.0, 0.0),
               {"type": "ground_truth", "scene_time_s": 0.15, "actors": [
                 {"role": "cut_in", "lateral_tracking_error_m": 0.9,
                  "speed_mps": 5.0, "target_speed_mps": 5.0}]},
               control(0.2, 1.0, 0.0)]
    summary = summarize_traffic_records(iter(records))
    self.assertTrue(summary["actor_tracking_valid"])
    self.assertGreater(summary["p95_lateral_tracking_error_m"], 0.4)

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
    baseline = {"vn_traffic_mode": "0", "seed": 42, "actor_tracking_valid": True,
                "run_valid": True,
                "runtime_artifact_sha256": {"driving_tinygrad.pkl.chunk0": "same"},
                "p95_abs_actual_jerk_mps3": 4.0, "accel_decel_switch_count": 3,
                "distance_m": 100.0, "contact_count": 0, "disengagement_count": 0,
                "minimum_observed_ttc_s": 2.0, "minimum_actor_clearance_m": 5.0, "data_valid": True,
                "exposures": [
                  {"actor_id": 1, "direction": "right_to_left", "entry_gap_m": 7,
                   "ego_speed_at_entry_mps": 8, "hold_s": 1.7, "complete": True, "evidence_missing": False},
                  {"actor_id": 2, "direction": "right_to_left", "entry_gap_m": 7.5,
                   "ego_speed_at_entry_mps": 8, "hold_s": 1.7, "complete": True, "evidence_missing": False},
                  {"actor_id": 3, "direction": "left_to_right", "entry_gap_m": 7,
                   "ego_speed_at_entry_mps": 8, "hold_s": 1.7, "complete": True, "evidence_missing": False},
                  {"actor_id": 4, "direction": "left_to_right", "entry_gap_m": 7.5,
                   "ego_speed_at_entry_mps": 8, "hold_s": 1.7, "complete": True, "evidence_missing": False},
                ]}
    candidate = {**baseline, "vn_traffic_mode": "1", "p95_abs_actual_jerk_mps3": 3.0, "distance_m": 96.0}
    result = compare_traffic_runs(baseline, candidate)
    self.assertTrue(result["comparison_valid"])
    self.assertTrue(result["phase2_improved"])
    self.assertTrue(result["pass"])
    contact_result = compare_traffic_runs(baseline, {**candidate, "contact_count": 1})
    self.assertTrue(contact_result["comparison_valid"])
    self.assertFalse(contact_result["phase2_improved"])
    self.assertFalse(contact_result["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "distance_m": 94.0})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "data_valid": False})["pass"])
    self.assertFalse(compare_traffic_runs({**baseline, "actual_accel_source": "carla"},
                                          {**candidate, "actual_accel_source": "speed_difference"})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "vn_traffic_mode": "0"})["pass"])
    self.assertFalse(compare_traffic_runs(baseline, {**candidate, "seed": 43})["pass"])
    self.assertTrue(compare_traffic_runs(baseline, {**candidate, "actor_tracking_valid": False})["pass"])
    self.assertFalse(compare_traffic_runs({**baseline, "configured_duration_s": 60, "duration_s": 0.4},
                                          {**candidate, "configured_duration_s": 60, "duration_s": 0.4})["pass"])

  def test_missing_provenance_keeps_comparison_invalid_even_with_better_jerk(self):
    baseline = {"vn_traffic_mode": "0", "run_valid": False, "data_valid": True,
                "actor_tracking_valid": True, "p95_abs_actual_jerk_mps3": 4.0,
                "accel_decel_switch_count": 2, "distance_m": 100.0,
                "contact_count": 0, "disengagement_count": 0}
    candidate = {**baseline, "vn_traffic_mode": "1", "p95_abs_actual_jerk_mps3": 2.0}
    result = compare_traffic_runs(baseline, candidate)
    self.assertFalse(result["comparison_valid"])
    self.assertFalse(result["phase2_improved"])
    self.assertFalse(result["pass"])

  def test_run_gate_requires_provenance_replay_and_both_cut_in_directions(self):
    summary = {"data_valid": True, "ground_truth_valid": True, "actor_tracking_valid": True,
               "cut_in_geometry_valid": True, "lead_attribution_valid": True, "exposure_valid": True,
               "completed_by_direction": {"right_to_left": 2, "left_to_right": 2},
               "duration_s": 60.0, "planner_trace_valid": True, "planner_output_complete": True,
               "mode_zero_parity_verified": True,
               "runtime_artifact_verified": True, "runtime_receipt_associated": True,
               "runtime_artifact_sha256": {"driving_tinygrad.pkl.chunk0": "same"},
               "compiled_source_link_verified": False,
               "scenario_start_monotonic_ns": 1_000_000_000,
               "scenario_finish_monotonic_ns": 61_000_000_000,
               "planner_trace_first_tick_ns": 999_000_000,
               "planner_trace_last_tick_ns": 61_001_000_000}
    manifest = {"duration_s": 60}
    self.assertTrue(assess_run(summary, manifest)["run_valid"])
    self.assertTrue(assess_run({**summary, "compiled_source_link_verified": False}, manifest)["run_valid"])
    self.assertFalse(assess_run({**summary, "planner_trace_valid": False}, manifest)["run_valid"])
    self.assertFalse(assess_run({**summary, "runtime_artifact_verified": False}, manifest)["run_valid"])
    self.assertFalse(assess_run({**summary, "runtime_receipt_associated": False}, manifest)["run_valid"])
    self.assertFalse(assess_run({**summary, "lead_attribution_valid": False}, manifest)["run_valid"])
    self.assertFalse(assess_run({**summary, "completed_by_direction": {"right_to_left": 4}}, manifest)["run_valid"])
    self.assertFalse(assess_run({**summary, "planner_trace_last_tick_ns": 50_000_000_000}, manifest)["run_valid"])

  def test_comparison_requires_identical_runtime_loaded_model_hashes(self):
    common = {"run_valid": True, "data_valid": True, "actual_accel_source": "carla",
              "p95_abs_actual_jerk_mps3": 4.0, "accel_decel_switch_count": 2,
              "distance_m": 100.0, "contact_count": 0, "disengagement_count": 0,
              "exposures": []}
    baseline = {**common, "vn_traffic_mode": "0",
                "runtime_artifact_sha256": {"driving_tinygrad.pkl.chunk0": "a"}}
    candidate = {**common, "vn_traffic_mode": "1", "p95_abs_actual_jerk_mps3": 2.0,
                 "runtime_artifact_sha256": {"driving_tinygrad.pkl.chunk0": "b"}}
    result = compare_traffic_runs(baseline, candidate)
    self.assertFalse(result["comparison_valid"])
    self.assertIn("runtime loaded model differs", result["validity_failures"])

  def test_stage_diagnostics_keep_model_radar_and_policy_separate(self):
    records = [
      {"type": "planner_input", "sequence": 0, "tick_time_ns": 1, "data": {
        "modelV2": {"leadsV3": [{"prob": 0.8}]},
        "radarState": {"leadOne": {"present": False}, "leadTwo": {"present": True}},
      }},
      {"type": "planner_output", "sequence": 0, "tick_time_ns": 2,
       "traffic_follow": {"stage": "closing_lead", "lead_index": 2}},
      {"type": "planner_input", "sequence": 1, "tick_time_ns": 3, "data": {
        "modelV2": {"leadsV3": []},
        "radarState": {"leadOne": {"present": True}, "leadTwo": {"present": False}},
      }},
      {"type": "planner_output", "sequence": 1, "tick_time_ns": 4,
       "traffic_follow": {"stage": "threat_cleared", "lead_index": None}},
    ]
    result = summarize_planner_stage_records(iter(records))
    self.assertEqual(result["model_candidate_present_count"], 1)
    self.assertEqual(result["radar_lead_one_present_count"], 1)
    self.assertEqual(result["radar_lead_two_present_count"], 1)
    self.assertEqual(result["traffic_follow_active_count"], 1)
    self.assertEqual(result["traffic_follow_selected_lead_two_count"], 1)
    self.assertTrue(result["planner_output_complete"])

  def test_runtime_receipt_must_precede_this_episode(self):
    summary = {"scenario_start_monotonic_ns": 200, "planner_trace_first_tick_ns": 150}
    self.assertTrue(runtime_receipt_matches_episode(summary, {"loaded_at_monotonic_ns": 100}))
    self.assertFalse(runtime_receipt_matches_episode(summary, {"loaded_at_monotonic_ns": 175}))
    self.assertFalse(runtime_receipt_matches_episode(summary, {}))

  def test_ground_truth_gap_is_not_hidden_by_complete_control_telemetry(self):
    summary = summarize_traffic_records(iter([
      control(0.0, 2.0, 0.0),
      {"type": "ground_truth", "scene_time_s": 0.0, "actors": []},
      control(0.1, 2.0, 0.0), control(0.2, 2.0, 0.0),
      {"type": "ground_truth", "scene_time_s": 1.0, "actors": []},
    ]))
    self.assertTrue(summary["data_valid"])
    self.assertFalse(summary["ground_truth_valid"])

  def test_ground_truth_must_cover_control_episode(self):
    records = [control(index / 10, 2.0, 0.0) for index in range(21)]
    records.extend({"type": "ground_truth", "scene_time_s": index / 2, "actors": []}
                   for index in range(2))
    summary = summarize_traffic_records(iter(records))
    self.assertTrue(summary["data_valid"])
    self.assertFalse(summary["ground_truth_valid"])


if __name__ == "__main__":
  unittest.main()
