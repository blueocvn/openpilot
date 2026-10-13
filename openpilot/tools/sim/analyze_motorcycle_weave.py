#!/usr/bin/env python3
"""Summarize a completed CARLA motorcycle-weave Phase 1 run."""

import argparse
import json
import math
from pathlib import Path


STOP_SPEED_MPS = 0.3
RESTART_SPEED_MPS = 1.0
DWELL_TIME_S = 1.0


def iter_report_records(report):
  """Read a legacy single file or all segments of a rotating scene report."""
  report = Path(report)
  segments = sorted((path for path in report.glob("events-*.jsonl") if path.stem[7:].isdigit()),
                    key=lambda path: int(path.stem[7:])) if report.is_dir() else []
  paths = [report] if report.is_file() else [report / "events.jsonl", *segments]
  for path in paths:
    with path.open(encoding="utf-8") as report_file:
      for line in report_file:
        if line.strip():
          yield json.loads(line)


def _p95(values):
  if not values:
    return None
  ordered = sorted(values)
  return ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]


def _stop_go_counts(samples):
  stopped = False
  below_since = None
  above_since = None
  stop_count = 0
  restart_count = 0
  for sample in samples:
    timestamp = sample["scene_time_s"]
    speed = sample["ego_speed_mps"]
    if not stopped:
      above_since = None
      if speed < STOP_SPEED_MPS:
        below_since = timestamp if below_since is None else below_since
        if timestamp - below_since >= DWELL_TIME_S:
          stopped = True
          stop_count += 1
          below_since = None
      else:
        below_since = None
    else:
      below_since = None
      if speed > RESTART_SPEED_MPS:
        above_since = timestamp if above_since is None else above_since
        if timestamp - above_since >= DWELL_TIME_S:
          stopped = False
          restart_count += 1
          above_since = None
      else:
        above_since = None
  return stop_count, restart_count


def match_radar_lead_actor(lead, actors):
  """Match observed lead geometry to one actor, or return unknown if ambiguous."""
  if not lead or not lead.get("present") or lead.get("distance_m") is None:
    return None
  candidates = [actor for actor in actors
                if actor.get("ego_forward_gap_m") is not None and actor.get("ego_lateral_offset_m") is not None
                and abs(actor["ego_forward_gap_m"] - lead["distance_m"]) <= 2.0
                and (lead.get("lateral_m") is None or
                     abs(actor["ego_lateral_offset_m"] - lead["lateral_m"]) <= 1.5)]
  return candidates[0].get("id") if len(candidates) == 1 else None


class _BoundedValues:
  """Retain a deterministic, bounded sample for long-run p95 estimates."""

  def __init__(self):
    self.values = []
    self.seen = 0
    self.stride = 1
    self.maximum = None

  def add(self, value):
    if value is None:
      return
    self.maximum = value if self.maximum is None else max(self.maximum, value)
    if self.seen % self.stride == 0:
      self.values.append(value)
      if len(self.values) > 10000:
        self.values = self.values[::2]
        self.stride *= 2
    self.seen += 1

  def p95(self):
    return _p95(self.values)


class _Evaluation:
  def __init__(self):
    self.stop_count = 0
    self.restart_count = 0
    self.stopped = False
    self.below_since = None
    self.above_since = None
    self.minimum_ego_speed = None
    self.minimum_clearance = None
    self.minimum_requested_accel = None
    self.lead_sample_count = 0
    self.braking_sample_count = 0
    self.maximum_ego_brake = 0.0
    self.tracking = _BoundedValues()
    self.longitudinal = _BoundedValues()
    self.lateral = _BoundedValues()
    self.speed_error = _BoundedValues()
    self.collisions = set()
    self.tracking_samples_after_contact_excluded = 0

  def add_control(self, sample):
    accel = sample.get("openpilot", {}).get("requested_accel_mps2")
    self.lead_sample_count += bool(sample.get("openpilot", {}).get("plan_has_lead", False))
    brake = sample.get("ego_brake", 0.0)
    self.braking_sample_count += brake > 0.05
    self.maximum_ego_brake = max(self.maximum_ego_brake, brake)
    if accel is not None:
      self.minimum_requested_accel = accel if self.minimum_requested_accel is None else min(self.minimum_requested_accel, accel)

  def add(self, sample, *, high_rate_controls=False, include_tracking=True):
    timestamp, speed = sample["scene_time_s"], sample["ego_speed_mps"]
    self.minimum_ego_speed = speed if self.minimum_ego_speed is None else min(self.minimum_ego_speed, speed)
    if not self.stopped:
      self.above_since = None
      if speed < STOP_SPEED_MPS:
        self.below_since = timestamp if self.below_since is None else self.below_since
        if timestamp - self.below_since >= DWELL_TIME_S:
          self.stopped = True
          self.stop_count += 1
          self.below_since = None
      else:
        self.below_since = None
    else:
      self.below_since = None
      if speed > RESTART_SPEED_MPS:
        self.above_since = timestamp if self.above_since is None else self.above_since
        if timestamp - self.above_since >= DWELL_TIME_S:
          self.stopped = False
          self.restart_count += 1
          self.above_since = None
      else:
        self.above_since = None
    if not high_rate_controls:
      self.add_control(sample)
    for actor in sample.get("actors", []):
      if actor.get("role", "cut_in") != "cut_in":
        continue
      clearance = actor.get("distance_to_ego_m")
      if clearance is not None:
        self.minimum_clearance = clearance if self.minimum_clearance is None else min(self.minimum_clearance, clearance)
      if not include_tracking:
        self.tracking_samples_after_contact_excluded += 1
        continue
      self.tracking.add(actor.get("tracking_error_m"))
      longitudinal_error = actor.get("longitudinal_tracking_error_m")
      lateral_error = actor.get("lateral_tracking_error_m")
      self.longitudinal.add(abs(longitudinal_error) if longitudinal_error is not None else None)
      self.lateral.add(abs(lateral_error) if lateral_error is not None else None)
      actor_speed = actor.get("speed_mps")
      self.speed_error.add(abs(actor_speed - actor.get("target_speed_mps", 6.0)) if actor_speed is not None else None)
    self.collisions.update((collision.get("other_actor_id"), collision.get("other_actor_type"))
                           for collision in sample.get("collisions", []))


def summarize_records(records):
  sample_count = 0
  first_sample_time = None
  last_sample_time = None
  first_cut_in_time = None
  completed_cut_ins = 0
  control_sample_count = 0
  high_rate_controls = False
  active_event = None
  cut_ins = []
  deferred_cut_in_count = 0
  cut_ins_with_lead = 0
  cut_ins_with_brake = 0
  cut_ins_with_collision = 0
  cut_ins_with_matched_lead = 0
  collision_event_count = 0
  first_collision_s = None
  scene_variant = None
  cut_in_started_count = 0
  cut_in_entered_count = 0
  active_cut_in_contact_ids = set()
  other_contact_ids = set()
  entry_gap_m = None
  entry_closing_speed_mps = None
  entry_ttc_s = None
  evaluation = _Evaluation()
  for record in records:
    if record.get("type") == "scenario_started":
      scene_variant = record.get("scene_variant", scene_variant)
    elif record.get("type") == "cut_in_started":
      cut_in_started_count += 1
      scene_variant = record.get("scene_variant", scene_variant)
      if first_cut_in_time is None:
        first_cut_in_time = record["scene_time_s"]
        evaluation = _Evaluation()
      active_event = {"actor_id": record.get("actor_id"), "lead": False, "matched_lead": False,
                      "brake": False, "collision": False, "direction": record.get("direction"),
                      "start_s": record["scene_time_s"], "entry_s": None, "entry_gap_m": None,
                      "host_start_ns": record.get("host_monotonic_ns"), "host_entry_ns": None,
                      "first_requested_decel_s": None, "first_actual_brake_s": None,
                      "verified": False}
      cut_ins.append(active_event)
    elif record.get("type") == "cut_in_entered_ego_lane":
      cut_in_entered_count += 1
      if entry_gap_m is None:
        entry_gap_m = record.get("actual_gap_m")
        entry_closing_speed_mps = record.get("closing_speed_mps")
        entry_ttc_s = record.get("entry_ttc_s")
      if active_event is not None and record.get("actor_id") == active_event["actor_id"]:
        active_event["entry_s"] = record["scene_time_s"]
        active_event["entry_gap_m"] = record.get("actual_gap_m")
        active_event["host_entry_ns"] = record.get("host_monotonic_ns")
    elif record.get("type") == "cut_in_finished":
      completed_cut_ins += 1
      if active_event is not None and record.get("actor_id") == active_event["actor_id"]:
        active_event["verified"] = active_event["entry_s"] is not None
        active_event["finish_s"] = record["scene_time_s"]
        cut_ins_with_lead += active_event["lead"]
        cut_ins_with_matched_lead += active_event["matched_lead"]
        cut_ins_with_brake += active_event["brake"]
        cut_ins_with_collision += active_event["collision"]
        active_event = None
    elif record.get("type") == "cut_in_deferred":
      deferred_cut_in_count += 1
    elif record.get("type") == "collision":
      collision_event_count += 1
      if first_collision_s is None:
        first_collision_s = record.get("scene_time_s")
      if first_cut_in_time is not None:
        evaluation.collisions.add((record.get("other_actor_id"), record.get("other_actor_type")))
      other_actor_id = record.get("other_actor_id")
      active_cut_in_contact = record.get("other_actor_role") == "active_cut_in" or (
        active_event is not None and other_actor_id == active_event["actor_id"])
      if other_actor_id is not None:
        (active_cut_in_contact_ids if active_cut_in_contact else other_contact_ids).add(other_actor_id)
      if active_event is not None and other_actor_id == active_event["actor_id"]:
        active_event["collision"] = True
    elif record.get("type") == "control_sample":
      high_rate_controls = True
      control_sample_count += 1
      if first_cut_in_time is not None:
        evaluation.add_control(record)
        if active_event is not None:
          active_event["lead"] |= bool(record.get("openpilot", {}).get("plan_has_lead", False))
          active_event["brake"] |= record.get("ego_brake", 0.0) > 0.05
          if (active_event["first_requested_decel_s"] is None and
              record.get("openpilot", {}).get("requested_accel_mps2", 0.0) < -0.1):
            active_event["first_requested_decel_s"] = record["scene_time_s"]
          if active_event["first_actual_brake_s"] is None and record.get("ego_brake", 0.0) > 0.05:
            active_event["first_actual_brake_s"] = record["scene_time_s"]
    elif record.get("type") == "ground_truth":
      sample_count += 1
      timestamp = record["scene_time_s"]
      first_sample_time = timestamp if first_sample_time is None else first_sample_time
      last_sample_time = timestamp
      if first_cut_in_time is None or timestamp >= first_cut_in_time:
        evaluation.add(record, high_rate_controls=high_rate_controls,
                       include_tracking=first_collision_s is None or timestamp < first_collision_s)
        if active_event is not None:
          matched_actor_id = match_radar_lead_actor(
            record.get("openpilot", {}).get("radar_lead_one"), record.get("actors", []))
          active_event["matched_lead"] |= matched_actor_id == active_event["actor_id"]
          if not high_rate_controls:
            active_event["lead"] |= bool(record.get("openpilot", {}).get("plan_has_lead", False))
            active_event["brake"] |= record.get("ego_brake", 0.0) > 0.05
          active_event["collision"] |= any(c.get("other_actor_id") == active_event["actor_id"]
                                           for c in record.get("collisions", []))
  if sample_count == 0:
    raise ValueError("report contains no ground_truth records")
  p95_lateral_error = evaluation.lateral.p95()
  maximum_lateral_error = evaluation.lateral.maximum
  p95_speed_error = evaluation.speed_error.p95()
  return {
    "duration_s": last_sample_time - first_sample_time,
    "sample_count": sample_count,
    "control_sample_count": control_sample_count,
    "evaluation_start_s": first_cut_in_time if first_cut_in_time is not None else first_sample_time,
    "stop_count": evaluation.stop_count,
    "restart_count": evaluation.restart_count,
    "minimum_ego_speed_mps": evaluation.minimum_ego_speed,
    "minimum_actor_clearance_m": evaluation.minimum_clearance,
    "minimum_requested_accel_mps2": evaluation.minimum_requested_accel,
    "lead_sample_count": evaluation.lead_sample_count,
    "braking_sample_count": evaluation.braking_sample_count,
    "maximum_ego_brake": evaluation.maximum_ego_brake,
    "p95_actor_tracking_error_m": evaluation.tracking.p95(),
    "maximum_actor_tracking_error_m": evaluation.tracking.maximum,
    "p95_longitudinal_tracking_error_m": evaluation.longitudinal.p95(),
    "p95_lateral_tracking_error_m": p95_lateral_error,
    "maximum_lateral_tracking_error_m": maximum_lateral_error,
    "p95_motorcycle_speed_error_mps": p95_speed_error,
    "actor_tracking_valid": (p95_lateral_error is not None and maximum_lateral_error is not None and
                             p95_speed_error is not None and p95_lateral_error <= 0.4 and
                             maximum_lateral_error <= 0.8 and p95_speed_error <= 1.0),
    "tracking_samples_after_contact_excluded": evaluation.tracking_samples_after_contact_excluded,
    "scene_variant": scene_variant,
    "cut_in_started_count": cut_in_started_count,
    "cut_in_entered_count": cut_in_entered_count,
    "active_cut_in_contact_count": len(active_cut_in_contact_ids),
    "other_contact_count": len(other_contact_ids),
    "entry_gap_m": entry_gap_m,
    "cut_ins": cut_ins,
    "deferred_cut_in_count": deferred_cut_in_count,
    "entry_closing_speed_mps": entry_closing_speed_mps,
    "entry_ttc_s": entry_ttc_s,
    "completed_cut_in_count": completed_cut_ins,
    "cut_ins_with_lead_count": cut_ins_with_lead,
    "cut_ins_with_matched_lead_count": cut_ins_with_matched_lead,
    "cut_ins_with_brake_count": cut_ins_with_brake,
    "cut_ins_with_collision_count": cut_ins_with_collision,
    "collision_count": len(evaluation.collisions),
    "collision_event_count": collision_event_count,
    "first_collision_s": first_collision_s,
  }


def summarize_report(report_dir):
  """Join each cut-in with planner evidence on the shared host monotonic clock."""
  report_dir = Path(report_dir)
  summary = summarize_records(iter_report_records(report_dir))
  planner_dir = report_dir / "planner"
  summary["planner_evidence_available"] = planner_dir.is_dir()
  for event in summary["cut_ins"]:
    event.update({"model_confirmed_before_entry_s": None,
                  "mpc_cutin_before_entry_s": None,
                  "planner_decel_before_entry_s": None,
                  "opposite_side_confirmation_count": 0,
                  "model_confirmation_attribution": None,
                  "mpc_source_attribution": None,
                  "planner_rejections": {}})
  if not planner_dir.is_dir():
    return summary
  for record in iter_report_records(planner_dir):
    if record.get("type") != "planner_output":
      continue
    tick_ns = record.get("tick_time_ns")
    if tick_ns is None:
      continue
    for event in summary["cut_ins"]:
      start_ns, entry_ns = event["host_start_ns"], event["host_entry_ns"]
      if start_ns is None or entry_ns is None or not start_ns <= tick_ns <= entry_ns:
        continue
      seconds_before_entry = (entry_ns - tick_ns) / 1e9
      diagnostic = record.get("cut_in") or {}
      reason = diagnostic.get("reason")
      if reason:
        event["planner_rejections"][reason] = event["planner_rejections"].get(reason, 0) + 1
      same_side = diagnostic.get("side") == event["direction"]
      if diagnostic.get("stage") == "confirmed" and not same_side:
        event["opposite_side_confirmation_count"] += 1
      if diagnostic.get("stage") == "confirmed" and same_side and event["model_confirmed_before_entry_s"] is None:
        event["model_confirmed_before_entry_s"] = seconds_before_entry
        event["model_confirmation_attribution"] = "side_compatible_not_actor_proven"
      if (same_side and str(record.get("mpc_source")) in ("3", "lead2") and
          event["mpc_cutin_before_entry_s"] is None):
        event["mpc_cutin_before_entry_s"] = seconds_before_entry
        event["mpc_source_attribution"] = "lowest_mpc_margin_not_causal_proof"
      accel = record.get("a_target_mps2")
      if accel is not None and accel < -0.1 and event["planner_decel_before_entry_s"] is None:
        event["planner_decel_before_entry_s"] = seconds_before_entry
  return summary


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("report", type=Path, help="events.jsonl or its containing report directory")
  args = parser.parse_args()

  report_path = args.report / "events.jsonl" if args.report.is_dir() else args.report
  summary = summarize_report(args.report) if args.report.is_dir() else summarize_records(iter_report_records(args.report))
  output_path = report_path.with_name("summary.json")
  output_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(summary, indent=2))
  print(f"Saved {output_path}")


if __name__ == "__main__":
  main()
