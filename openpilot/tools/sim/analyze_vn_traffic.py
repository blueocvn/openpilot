#!/usr/bin/env python3
"""Stream Phase 2 comfort/safety metrics from one or two CARLA reports."""

import argparse
import json
import math
from pathlib import Path

from openpilot.tools.sim.analyze_motorcycle_weave import _BoundedValues, iter_report_records, match_radar_lead_actor
from openpilot.tools.sim.planner_trace import parity_report_valid, trace_health
from openpilot.tools.sim.model_provenance import verify_compiled_source_link, verify_runtime_artifact
from openpilot.tools.sim.cut_in_exposure import ExposureCollector
from openpilot.tools.sim.exposure_matching import match_exposures


class _SwitchCounter:
  def __init__(self):
    self.state = None
    self.pending = None
    self.pending_since = None
    self.count = 0

  def add(self, timestamp: float, acceleration: float) -> None:
    candidate = 1 if acceleration >= 0.2 else -1 if acceleration <= -0.2 else None
    if candidate is None or candidate == self.state:
      self.pending = None
      return
    if candidate != self.pending:
      self.pending, self.pending_since = candidate, timestamp
    elif timestamp - self.pending_since >= 0.3 - 1e-6:
      if self.state is not None:
        self.count += 1
      self.state = candidate
      self.pending = None


def classify_lead(lead, actors, active_cut_in_id):
  if not lead or not lead.get("present"):
    return "no_lead_observed"
  actor_id = match_radar_lead_actor(lead, actors)
  if actor_id is None:
    return "ambiguous"
  matched = next(actor for actor in actors if actor.get("id") == actor_id)
  return "matched_cut_in" if actor_id == active_cut_in_id and matched.get("role") == "cut_in" else "matched_other_actor"


def summarize_traffic_records(records):
  records = iter(records)
  collector = ExposureCollector()
  actual_jerk, requested_jerk = _BoundedValues(), _BoundedValues()
  lateral_error, motorcycle_speed_error = _BoundedValues(), _BoundedValues()
  actual_switches, requested_switches = _SwitchCounter(), _SwitchCounter()
  first_time = last_time = first_contact_s = None
  previous_control = None
  previous_actual_accel = previous_requested = previous_comfort_time = None
  previous_active = None
  distance_m = 0.0
  control_count = active_control_count = comfort_count = contact_event_count = disengagement_count = control_gap_count = 0
  contact_actor_ids = set()
  unknown_contact_count = 0
  minimum_gap = minimum_ttc = None
  stop_count = restart_count = 0
  stopped = False
  stop_since = restart_since = None
  invalid_sample_count = 0
  direct_accel_count = estimated_accel_count = 0
  ground_truth_count = ground_truth_gap_count = 0
  first_ground_truth_time = previous_ground_truth_time = None
  completed_by_direction = {"right_to_left": 0, "left_to_right": 0}
  entry_gaps, lane_holds = [], []
  active_cut_in_id = None
  attribution = dict.fromkeys(("matched_cut_in", "matched_other_actor", "no_lead_observed", "ambiguous"), 0)
  actor_snapshot_count = fresh_openpilot_snapshot_count = active_cut_in_control_count = 0
  scenario_start_monotonic_ns = scenario_finish_monotonic_ns = None

  for record in records:
    collector.add(record)
    kind = record.get("type")
    if kind == "scenario_started":
      scenario_start_monotonic_ns = record.get("host_monotonic_ns")
    if kind == "scenario_finished":
      scenario_finish_monotonic_ns = record.get("host_monotonic_ns")
    if kind == "cut_in_started":
      active_cut_in_id = record.get("actor_id")
    if kind == "cut_in_finished" and record.get("direction") in completed_by_direction:
      completed_by_direction[record["direction"]] += 1
      if record.get("actor_id") == active_cut_in_id:
        active_cut_in_id = None
    if kind == "cut_in_entered_ego_lane":
      gap = record.get("actual_gap_m")
      if gap is not None and math.isfinite(gap):
        entry_gaps.append(gap)
    if kind == "cut_in_left_ego_lane":
      hold = record.get("actual_hold_s")
      if hold is not None and math.isfinite(hold):
        lane_holds.append(hold)
    if kind == "collision":
      contact_event_count += 1
      actor_id = record.get("other_actor_id")
      if actor_id is None:
        unknown_contact_count += 1
      else:
        contact_actor_ids.add(actor_id)
      if first_contact_s is None:
        first_contact_s = record.get("scene_time_s")
    if kind == "ground_truth":
      ground_truth_count += 1
      gt_time = record.get("scene_time_s")
      if gt_time is None or not math.isfinite(gt_time) or (previous_ground_truth_time is not None and
          not 0.45 <= gt_time - previous_ground_truth_time <= 0.55):
        ground_truth_gap_count += 1
      if first_ground_truth_time is None:
        first_ground_truth_time = gt_time
      previous_ground_truth_time = gt_time
      for actor in record.get("actors", []):
        if actor.get("role", "cut_in") != "cut_in":
          continue
        gap = actor.get("distance_to_ego_m")
        if gap is not None and math.isfinite(gap):
          minimum_gap = gap if minimum_gap is None else min(minimum_gap, gap)
        if first_contact_s is None or record.get("scene_time_s", math.inf) < first_contact_s:
          lateral = actor.get("lateral_tracking_error_m")
          if lateral is not None and math.isfinite(lateral):
            lateral_error.add(abs(lateral))
          speed, target_speed = actor.get("speed_mps"), actor.get("target_speed_mps")
          if speed is not None and target_speed is not None and math.isfinite(speed) and math.isfinite(target_speed):
            motorcycle_speed_error.add(abs(speed - target_speed))
    if kind not in ("ground_truth", "control_sample"):
      continue
    lead = record.get("openpilot", {}).get("radar_lead_one") or {}
    gap, relative_speed = lead.get("distance_m"), lead.get("relative_speed_mps")
    if lead.get("present") and gap is not None and relative_speed is not None and gap >= 0 and relative_speed < 0:
      ttc = gap / -relative_speed
      if math.isfinite(ttc):
        minimum_ttc = ttc if minimum_ttc is None else min(minimum_ttc, ttc)
    if kind != "control_sample":
      continue

    if active_cut_in_id is not None:
      active_cut_in_control_count += 1
      if "actors" in record:
        actor_snapshot_count += 1
      sample_ns = record.get("host_monotonic_ns")
      snapshot_ns = record.get("openpilot", {}).get("snapshot_monotonic_ns")
      if (isinstance(sample_ns, int) and isinstance(snapshot_ns, int) and
          0 <= sample_ns - snapshot_ns <= 250_000_000):
        fresh_openpilot_snapshot_count += 1
      status = classify_lead(record.get("openpilot", {}).get("radar_lead_one"),
                             record.get("actors", []), active_cut_in_id)
      attribution[status] += 1

    timestamp, speed = record.get("scene_time_s"), record.get("ego_speed_mps")
    request = record.get("openpilot", {}).get("requested_accel_mps2")
    if timestamp is None or speed is None or request is None or not all(map(math.isfinite, (timestamp, speed, request))):
      invalid_sample_count += 1
      continue
    control_count += 1
    first_time = timestamp if first_time is None else first_time
    last_time = timestamp
    active = bool(record.get("openpilot", {}).get("long_active", False))
    active_control_count += active
    if previous_active is True and not active:
      disengagement_count += 1
    previous_active = active

    if previous_control is not None:
      previous_time, previous_speed = previous_control
      dt = timestamp - previous_time
      if dt <= 0 or dt > 0.15:
        control_gap_count += 1
      if dt > 0:
        distance_m += (previous_speed + speed) * dt / 2
    else:
      dt = None
    previous_control = (timestamp, speed)

    if not stopped:
      restart_since = None
      if speed < 0.3:
        stop_since = timestamp if stop_since is None else stop_since
        if timestamp - stop_since >= 1.0:
          stopped, stop_count, stop_since = True, stop_count + 1, None
      else:
        stop_since = None
    else:
      stop_since = None
      if speed > 1.0:
        restart_since = timestamp if restart_since is None else restart_since
        if timestamp - restart_since >= 1.0:
          stopped, restart_count, restart_since = False, restart_count + 1, None
      else:
        restart_since = None

    if first_contact_s is not None and timestamp >= first_contact_s:
      continue
    if dt is not None and dt <= 0:
      continue
    comfort_count += 1
    requested_switches.add(timestamp, request)
    if previous_requested is not None and previous_comfort_time is not None:
      jerk = (request - previous_requested) / (timestamp - previous_comfort_time)
      requested_jerk.add(abs(jerk))
    previous_requested = request
    direct_accel = record.get("ego_accel_mps2")
    if direct_accel is not None and math.isfinite(direct_accel):
      actual_accel = direct_accel
      direct_accel_count += 1
    elif dt is not None and dt > 0:
      actual_accel = (speed - previous_speed) / dt
      estimated_accel_count += 1
    else:
      actual_accel = None
    if actual_accel is not None:
      actual_switches.add(timestamp, actual_accel)
      if previous_actual_accel is not None and previous_comfort_time is not None:
        actual_jerk.add(abs((actual_accel - previous_actual_accel) / (timestamp - previous_comfort_time)))
      previous_actual_accel = actual_accel
    previous_comfort_time = timestamp

  exposures = collector.finish()
  invalid_exposure_ids = [event.get("actor_id") for event in exposures
                          if event.get("evidence_missing") or not event.get("complete")]
  valid_completed_by_direction = {direction: sum(1 for event in exposures if event.get("direction") == direction and
                                                 event.get("complete") and not event.get("evidence_missing"))
                                  for direction in completed_by_direction}
  return {
    "data_valid": (control_count >= 3 and active_control_count / control_count >= 0.9 and
                   control_gap_count == 0 and invalid_sample_count == 0),
    "control_sample_count": control_count, "active_control_sample_count": active_control_count,
    "ground_truth_count": ground_truth_count, "ground_truth_gap_count": ground_truth_gap_count,
    "ground_truth_valid": (ground_truth_count >= 2 and ground_truth_gap_count == 0 and
                           first_time is not None and last_time is not None and
                           first_ground_truth_time is not None and previous_ground_truth_time is not None and
                           first_ground_truth_time <= first_time + 0.6 and
                           previous_ground_truth_time >= last_time - 0.6),
    "scenario_start_monotonic_ns": scenario_start_monotonic_ns,
    "scenario_finish_monotonic_ns": scenario_finish_monotonic_ns,
    "completed_by_direction": valid_completed_by_direction,
    "finished_callback_by_direction": completed_by_direction,
    "invalid_exposure_ids": invalid_exposure_ids,
    "exposure_valid": bool(exposures) and not invalid_exposure_ids,
    "lead_attribution_counts": attribution,
    "lead_attribution_valid": (active_cut_in_control_count > 0 and
                               actor_snapshot_count == active_cut_in_control_count and
                               fresh_openpilot_snapshot_count == active_cut_in_control_count),
    "fresh_openpilot_snapshot_count": fresh_openpilot_snapshot_count,
    "lead_attribution_resolved_fraction": (
      (attribution["matched_cut_in"] + attribution["matched_other_actor"]) /
      (attribution["matched_cut_in"] + attribution["matched_other_actor"] + attribution["ambiguous"])
      if attribution["matched_cut_in"] + attribution["matched_other_actor"] + attribution["ambiguous"] else None),
    "entry_gap_count": len(entry_gaps), "lane_hold_count": len(lane_holds),
    # These are measured scene targets, diagnostics rather than a post-hoc gate.
    "cut_in_geometry_valid": bool(entry_gaps) and bool(lane_holds),
    "comfort_sample_count": comfort_count,
    "control_gap_count": control_gap_count, "invalid_sample_count": invalid_sample_count,
    "duration_s": last_time - first_time if first_time is not None else None,
    "distance_m": distance_m,
    "mean_speed_mps": distance_m / (last_time - first_time) if first_time is not None and last_time > first_time else None,
    "p95_abs_actual_jerk_mps3": actual_jerk.p95(),
    "actual_accel_source": ("mixed" if direct_accel_count and estimated_accel_count else
                            "carla" if direct_accel_count else "speed_difference" if estimated_accel_count else None),
    "p95_abs_requested_jerk_mps3": requested_jerk.p95(),
    "accel_decel_switch_count": actual_switches.count,
    "requested_switch_count": requested_switches.count,
    "stop_count": stop_count, "restart_count": restart_count,
    "contact_count": len(contact_actor_ids) + unknown_contact_count,
    "contact_event_count": contact_event_count, "disengagement_count": disengagement_count,
    "first_contact_s": first_contact_s,
    "comfort_excluded_duration_s": max(0.0, last_time - first_contact_s) if first_contact_s is not None and last_time is not None else 0.0,
    "minimum_actor_clearance_m": minimum_gap,
    "minimum_observed_ttc_s": minimum_ttc,
    "p95_lateral_tracking_error_m": lateral_error.p95(),
    "maximum_lateral_tracking_error_m": lateral_error.maximum,
    "p95_motorcycle_speed_error_mps": motorcycle_speed_error.p95(),
    "actor_tracking_valid": (lateral_error.p95() is not None and motorcycle_speed_error.p95() is not None and
                             motorcycle_speed_error.p95() <= 1.0),
    "exposures": exposures,
  }


def assess_run(summary, manifest):
  failures = []
  for key in ("data_valid", "ground_truth_valid", "cut_in_geometry_valid", "lead_attribution_valid", "exposure_valid"):
    if not summary.get(key):
      failures.append(f"{key} failed")
  counts = summary.get("completed_by_direction") or {}
  if any(counts.get(direction, 0) < 2 for direction in ("right_to_left", "left_to_right")):
    failures.append("fewer than two completed cut-ins per direction")
  expected, actual = manifest.get("duration_s"), summary.get("duration_s")
  if expected is None or expected <= 0 or actual is None or abs(actual - expected) > max(0.25, expected * 0.01):
    failures.append("finite episode did not complete configured duration")
  if summary.get("compiled_source_link_verified") is not True:
    failures.append("compiled_source_link_verified not verified")
  start, finish = summary.get("scenario_start_monotonic_ns"), summary.get("scenario_finish_monotonic_ns")
  first_tick, last_tick = summary.get("planner_trace_first_tick_ns"), summary.get("planner_trace_last_tick_ns")
  if (not all(isinstance(value, int) for value in (start, finish, first_tick, last_tick)) or
      start >= finish or first_tick > start + 100_000_000 or last_tick < finish - 100_000_000):
    failures.append("planner trace does not cover full episode")
  for key in ("runtime_artifact_verified", "planner_trace_valid", "mode_zero_parity_verified"):
    if summary.get(key) is not True:
      failures.append(f"{key} not verified")
  return {"run_valid": not failures, "run_validity_failures": failures}


def summarize_exposure_windows(records, exposures):
  """Stream compact metrics for unioned measured-entry/exit windows.

  Windows run from one second before entry until two seconds after exit; an
  overlap is counted once and samples at/after first contact are excluded.
  """
  intervals = []
  for event in exposures:
    if event.get("complete") and event.get("entry_time_s") is not None and event.get("exit_time_s") is not None:
      intervals.append((max(0.0, event["entry_time_s"] - 1.0), event["exit_time_s"] + 2.0))
  merged = []
  for start, end in sorted(intervals):
    if merged and start <= merged[-1][1]:
      merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    else:
      merged.append((start, end))
  first_contact = None
  samples = brake_samples = 0
  for record in records:
    if record.get("type") == "collision" and first_contact is None:
      first_contact = record.get("scene_time_s")
    if record.get("type") != "control_sample":
      continue
    timestamp = record.get("scene_time_s")
    if timestamp is None or (first_contact is not None and timestamp >= first_contact):
      continue
    if any(start <= timestamp <= end for start, end in merged):
      samples += 1
      brake_samples += bool(record.get("ego_brake", 0.0) > 0.01)
  return {"window_count": len(merged), "control_sample_count": samples,
          "brake_sample_count": brake_samples, "first_contact_s": first_contact}


def compare_traffic_runs(baseline, candidate):
  validity_failures = []
  outcome_failures = []
  if baseline.get("vn_traffic_mode") != "0" or candidate.get("vn_traffic_mode") != "1":
    validity_failures.append("expected mode 0 baseline and mode 1 candidate")
  for key in ("git_commit", "scene", "case", "seed", "configured_duration_s", "carla_map",
              "carla_server_version", "params", "source_sha256", "onnx_sha256", "compiled_artifact_sha256"):
    if baseline.get(key) != candidate.get(key):
      validity_failures.append(f"run setting differs: {key}")
  for label, run in (("baseline", baseline), ("candidate", candidate)):
    expected, actual = run.get("configured_duration_s"), run.get("duration_s")
    if expected is not None and expected > 0 and (actual is None or abs(actual - expected) > max(0.25, expected * 0.01)):
      validity_failures.append(f"{label} did not complete configured duration")
  if not baseline.get("run_valid") or not candidate.get("run_valid"):
    validity_failures.append("one or both runs failed evidence gates")
  if not baseline.get("data_valid") or not candidate.get("data_valid"):
    validity_failures.append("missing or irregular control telemetry")
  matching = match_exposures(baseline.get("exposures") or [], candidate.get("exposures") or [])
  if not matching["comparison_valid"]:
    validity_failures.append("fewer than two comparable exposures per direction")
  baseline_source, candidate_source = baseline.get("actual_accel_source"), candidate.get("actual_accel_source")
  if baseline_source != candidate_source or "mixed" in (baseline_source, candidate_source):
    validity_failures.append("actual acceleration sources differ or are mixed")
  baseline_jerk, candidate_jerk = baseline.get("p95_abs_actual_jerk_mps3"), candidate.get("p95_abs_actual_jerk_mps3")
  if baseline_jerk is None or candidate_jerk is None or candidate_jerk > 0.85 * baseline_jerk:
    outcome_failures.append("actual jerk did not improve by 15%")
  if candidate.get("accel_decel_switch_count", math.inf) > baseline.get("accel_decel_switch_count", -1):
    outcome_failures.append("accel/decel switches increased")
  if candidate.get("distance_m", -1) < 0.95 * baseline.get("distance_m", math.inf):
    outcome_failures.append("distance fell below 95% of baseline")
  for key in ("contact_count", "disengagement_count"):
    if candidate.get(key, math.inf) > baseline.get(key, -1):
      outcome_failures.append(f"{key} increased")
  for key, tolerance in (("minimum_observed_ttc_s", 0.2), ("minimum_actor_clearance_m", 0.25)):
    old, new = baseline.get(key), candidate.get(key)
    if (old is None) != (new is None) or (old is not None and new < old - tolerance):
      outcome_failures.append(f"{key} worsened")
  comparison_valid = not validity_failures
  phase2_improved = comparison_valid and not outcome_failures
  outcome_keys = ("p95_abs_actual_jerk_mps3", "distance_m", "contact_count", "minimum_observed_ttc_s",
                  "minimum_actor_clearance_m")
  return {"comparison_valid": comparison_valid, "phase2_improved": phase2_improved,
          "pass": phase2_improved, "validity_failures": validity_failures,
          "outcome_failures": outcome_failures, "failures": validity_failures + outcome_failures,
          "baseline_had_contact": baseline.get("contact_count", 0) > 0,
          "exposure_matching": matching,
          "full_run_outcomes": {"baseline": {key: baseline.get(key) for key in outcome_keys},
                                "candidate": {key: candidate.get(key) for key in outcome_keys}},
          "matched_event_window_outcomes": {"baseline": baseline.get("window_outcomes"), "candidate": candidate.get("window_outcomes")}}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("reports", type=Path, nargs="+", help="one report to summarize, or baseline and candidate to compare")
  args = parser.parse_args()
  if len(args.reports) not in (1, 2):
    parser.error("pass one or two report directories")
  summaries = [summarize_traffic_records(iter_report_records(path)) for path in args.reports]
  for path, summary in zip(args.reports, summaries, strict=True):
    summary["window_outcomes"] = summarize_exposure_windows(iter_report_records(path), summary["exposures"])
    summary.update(trace_health(path))
    summary["mode_zero_parity_verified"] = parity_report_valid(path)
    manifest_path = path / "manifest.json"
    if manifest_path.exists():
      manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
      try:
        receipt = json.loads((path / "model-runtime.json").read_text(encoding="utf-8"))
      except (OSError, ValueError):
        receipt = {}
      try:
        build_receipt = json.loads((path / "model-source-build.json").read_text(encoding="utf-8"))
      except (OSError, ValueError):
        build_receipt = {}
      summary["runtime_artifact_verified"] = verify_runtime_artifact(manifest, receipt)
      summary["compiled_source_link_verified"], summary["compiled_source_link_reason"] = verify_compiled_source_link(
        manifest, build_receipt, receipt)
      summary["vn_traffic_mode"] = manifest.get("vn_traffic_mode", "unknown")
      summary["vn_traffic_profile"] = manifest.get("vn_traffic_profile", "unknown")
      summary["configured_duration_s"] = manifest.get("duration_s")
      for key in ("git_commit", "scene", "case", "seed", "carla_map",
                  "carla_server_version", "params", "source_sha256", "onnx_sha256", "compiled_artifact_sha256"):
        summary[key] = manifest.get(key)
      summary.update(assess_run(summary, manifest))
    else:
      summary["runtime_artifact_verified"] = False
      summary.update(assess_run(summary, {}))
  result = {"runs": summaries}
  if len(summaries) == 2:
    result["comparison"] = compare_traffic_runs(*summaries)
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()
