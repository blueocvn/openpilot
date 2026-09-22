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

  def add_control(self, sample):
    accel = sample.get("openpilot", {}).get("requested_accel_mps2")
    self.lead_sample_count += bool(sample.get("openpilot", {}).get("plan_has_lead", False))
    brake = sample.get("ego_brake", 0.0)
    self.braking_sample_count += brake > 0.05
    self.maximum_ego_brake = max(self.maximum_ego_brake, brake)
    if accel is not None:
      self.minimum_requested_accel = accel if self.minimum_requested_accel is None else min(self.minimum_requested_accel, accel)

  def add(self, sample, *, high_rate_controls=False):
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
  cut_ins_with_lead = 0
  cut_ins_with_brake = 0
  cut_ins_with_collision = 0
  evaluation = _Evaluation()
  for record in records:
    if record.get("type") == "cut_in_started":
      if first_cut_in_time is None:
        first_cut_in_time = record["scene_time_s"]
        evaluation = _Evaluation()
      active_event = {"actor_id": record.get("actor_id"), "lead": False, "brake": False, "collision": False}
    elif record.get("type") == "cut_in_finished":
      completed_cut_ins += 1
      if active_event is not None and record.get("actor_id") == active_event["actor_id"]:
        cut_ins_with_lead += active_event["lead"]
        cut_ins_with_brake += active_event["brake"]
        cut_ins_with_collision += active_event["collision"]
        active_event = None
    elif record.get("type") == "control_sample":
      high_rate_controls = True
      control_sample_count += 1
      if first_cut_in_time is not None:
        evaluation.add_control(record)
        if active_event is not None:
          active_event["lead"] |= bool(record.get("openpilot", {}).get("plan_has_lead", False))
          active_event["brake"] |= record.get("ego_brake", 0.0) > 0.05
    elif record.get("type") == "ground_truth":
      sample_count += 1
      timestamp = record["scene_time_s"]
      first_sample_time = timestamp if first_sample_time is None else first_sample_time
      last_sample_time = timestamp
      if first_cut_in_time is None or timestamp >= first_cut_in_time:
        evaluation.add(record, high_rate_controls=high_rate_controls)
        if active_event is not None:
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
    "completed_cut_in_count": completed_cut_ins,
    "cut_ins_with_lead_count": cut_ins_with_lead,
    "cut_ins_with_brake_count": cut_ins_with_brake,
    "cut_ins_with_collision_count": cut_ins_with_collision,
    "collision_count": len(evaluation.collisions),
  }


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("report", type=Path, help="events.jsonl or its containing report directory")
  args = parser.parse_args()

  report_path = args.report / "events.jsonl" if args.report.is_dir() else args.report
  summary = summarize_records(iter_report_records(args.report))
  output_path = report_path.with_name("summary.json")
  output_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(summary, indent=2))
  print(f"Saved {output_path}")


if __name__ == "__main__":
  main()
