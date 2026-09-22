#!/usr/bin/env python3
"""Stream Phase 2 comfort/safety metrics from one or two CARLA reports."""

import argparse
import json
import math
from pathlib import Path

from openpilot.tools.sim.analyze_motorcycle_weave import _BoundedValues, iter_report_records


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


def summarize_traffic_records(records):
  actual_jerk, requested_jerk = _BoundedValues(), _BoundedValues()
  actual_switches, requested_switches = _SwitchCounter(), _SwitchCounter()
  first_time = last_time = first_contact_s = None
  previous_control = None
  previous_actual_accel = previous_requested = previous_comfort_time = None
  previous_active = None
  distance_m = 0.0
  control_count = comfort_count = contact_event_count = disengagement_count = control_gap_count = 0
  contact_actor_ids = set()
  unknown_contact_count = 0
  minimum_gap = minimum_ttc = None
  stop_count = restart_count = 0
  stopped = False
  stop_since = restart_since = None
  invalid_sample_count = 0
  direct_accel_count = estimated_accel_count = 0

  for record in records:
    kind = record.get("type")
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
      for actor in record.get("actors", []):
        gap = actor.get("distance_to_ego_m") if actor.get("role", "cut_in") == "cut_in" else None
        if gap is not None and math.isfinite(gap):
          minimum_gap = gap if minimum_gap is None else min(minimum_gap, gap)
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

    timestamp, speed = record.get("scene_time_s"), record.get("ego_speed_mps")
    request = record.get("openpilot", {}).get("requested_accel_mps2")
    if timestamp is None or speed is None or request is None or not all(map(math.isfinite, (timestamp, speed, request))):
      invalid_sample_count += 1
      continue
    control_count += 1
    first_time = timestamp if first_time is None else first_time
    last_time = timestamp
    active = bool(record.get("openpilot", {}).get("long_active", False))
    if previous_active is True and not active:
      disengagement_count += 1
    previous_active = active

    if previous_control is not None:
      previous_time, previous_speed = previous_control
      dt = timestamp - previous_time
      if dt <= 0 or dt > 0.25:
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

  return {
    "data_valid": control_count >= 3 and control_gap_count == 0 and invalid_sample_count == 0,
    "control_sample_count": control_count, "comfort_sample_count": comfort_count,
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
  }


def compare_traffic_runs(baseline, candidate):
  failures = []
  if baseline.get("vn_traffic_mode") != "0" or candidate.get("vn_traffic_mode") != "1":
    failures.append("expected mode 0 baseline and mode 1 candidate")
  for key in ("git_commit", "scene", "case", "seed", "configured_duration_s", "carla_map",
              "carla_server_version", "params", "source_sha256", "onnx_sha256", "compiled_artifact_sha256"):
    if baseline.get(key) != candidate.get(key):
      failures.append(f"run setting differs: {key}")
  if not baseline.get("data_valid") or not candidate.get("data_valid"):
    failures.append("missing or irregular control telemetry")
  baseline_source, candidate_source = baseline.get("actual_accel_source"), candidate.get("actual_accel_source")
  if baseline_source != candidate_source or "mixed" in (baseline_source, candidate_source):
    failures.append("actual acceleration sources differ or are mixed")
  baseline_jerk, candidate_jerk = baseline.get("p95_abs_actual_jerk_mps3"), candidate.get("p95_abs_actual_jerk_mps3")
  if baseline_jerk is None or candidate_jerk is None or candidate_jerk > 0.85 * baseline_jerk:
    failures.append("actual jerk did not improve by 15%")
  if candidate.get("accel_decel_switch_count", math.inf) > baseline.get("accel_decel_switch_count", -1):
    failures.append("accel/decel switches increased")
  if candidate.get("distance_m", -1) < 0.95 * baseline.get("distance_m", math.inf):
    failures.append("distance fell below 95% of baseline")
  for key in ("contact_count", "disengagement_count"):
    if candidate.get(key, math.inf) > baseline.get(key, -1):
      failures.append(f"{key} increased")
  for key, tolerance in (("minimum_observed_ttc_s", 0.2), ("minimum_actor_clearance_m", 0.25)):
    old, new = baseline.get(key), candidate.get(key)
    if (old is None) != (new is None) or (old is not None and new < old - tolerance):
      failures.append(f"{key} worsened")
  return {"pass": not failures, "failures": failures,
          "baseline_had_contact": baseline.get("contact_count", 0) > 0}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("reports", type=Path, nargs="+", help="one report to summarize, or baseline and candidate to compare")
  args = parser.parse_args()
  if len(args.reports) not in (1, 2):
    parser.error("pass one or two report directories")
  summaries = [summarize_traffic_records(iter_report_records(path)) for path in args.reports]
  for path, summary in zip(args.reports, summaries, strict=True):
    manifest_path = path / "manifest.json"
    if manifest_path.exists():
      manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
      summary["vn_traffic_mode"] = manifest.get("vn_traffic_mode", "unknown")
      summary["vn_traffic_profile"] = manifest.get("vn_traffic_profile", "unknown")
      summary["configured_duration_s"] = manifest.get("duration_s")
      for key in ("git_commit", "scene", "case", "seed", "carla_map",
                  "carla_server_version", "params", "source_sha256", "onnx_sha256", "compiled_artifact_sha256"):
        summary[key] = manifest.get(key)
  result = {"runs": summaries}
  if len(summaries) == 2:
    result["comparison"] = compare_traffic_runs(*summaries)
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()
