#!/usr/bin/env python3
"""Summarise a benchmark directory: one JSON result per run -> pass/fail per run, a table per scenario, a CSV.

  python3 tools/sim/bridge/bench_summary.py <dir with *.json> [--csv runs.csv] [--markdown table.md]

The scenario label of a run is "name/town/set_kph" (LEAD_SCENARIO, set by bench.sh). The pass rules are the
acceptance table of plans/261112-1710-static-obstacle-braking-experiments/plan.md. Standard library only.
"""
import argparse
import csv
import json
import pathlib
import statistics
import sys

MIN_GAP_M = 4.0
MAX_DECEL_MPS2 = -3.5
FOLLOW_TOLERANCE = 0.2
CLEAR_ROAD_SPEED_FRACTION = 0.9
OVERTAKE_BRAKE_MAX = 0.05     # brake pedal fraction that still counts as "not braking against the driver"

MIN_REALTIME_FACTOR = 0.95    # below this the machine could not keep up and openpilot saw the world in slow motion

OBSTACLE_SCENARIOS = ("stationary-motorcycle", "slow-motorcycle", "parked-car", "barrier", "container")


def split_label(label):
  """'parked-car/Town05_Opt/30' -> ('parked-car', 'Town05_Opt', 30.0)."""
  name, town, kph = (label.split("/") + ["", "", "0"])[:3]
  try:
    return name, town, float(kph)
  except ValueError:
    return name, town, 0.0


def judge(run):
  """(passed, reason) for one run result."""
  name, _, set_kph = split_label(run.get("scenario", ""))
  if not run.get("engaged"):
    return False, "never engaged"
  if run.get("realtime_factor", 1.0) < MIN_REALTIME_FACTOR:
    return False, f"overloaded machine (realtime factor {run['realtime_factor']}): discard and rerun"
  if run.get("other_contacts", 0) > 0 and not run.get("collision"):
    return False, "left the road (hit scenery)"
  if name in OBSTACLE_SCENARIOS:
    if run.get("collision"):
      return False, f"collision at {run.get('impact_speed_mps')} m/s"
    gap = run.get("min_gap_m")
    if gap is None:
      return False, "obstacle never ahead"
    if gap < MIN_GAP_M:
      return False, f"min gap {gap} m"
    if run.get("peak_decel_mps2", 0.0) < MAX_DECEL_MPS2:
      return False, f"peak decel {run['peak_decel_mps2']}"
    return True, ""
  if name == "follow-motorcycle":
    steady, intended = run.get("steady_gap_m"), run.get("intended_gap_m")
    if run.get("collision"):
      return False, "collision"
    if steady is None or intended is None:
      return False, "no steady follow"
    if abs(steady - intended) > FOLLOW_TOLERANCE * intended:
      return False, f"gap {steady} m against intended {intended} m"
    return True, ""
  if name == "clear-road":
    avg = run.get("avg_speed_cruise_mps")
    if run.get("false_brake_events", 0) > 0:
      return False, f"{run['false_brake_events']} false braking events"
    if avg is None or avg < CLEAR_ROAD_SPEED_FRACTION * set_kph / 3.6:
      return False, f"average speed {avg} m/s"
    return True, ""
  if name == "overtake":
    if run.get("collision"):
      return False, "contact"
    if run.get("overtake_max_brake", 0.0) > OVERTAKE_BRAKE_MAX:
      return False, f"braked against the input ({run['overtake_max_brake']})"
    return True, ""
  return False, f"unknown scenario '{name}'"


def load_runs(directory):
  runs = []
  for path in sorted(pathlib.Path(directory).glob("*.json")):
    try:
      run = json.loads(path.read_text())
    except json.JSONDecodeError:
      run = {"scenario": path.stem, "engaged": False}
    run["file"] = path.name
    run["passed"], run["reason"] = judge(run)
    runs.append(run)
  return runs


def group(runs):
  groups = {}
  for run in runs:
    groups.setdefault(run.get("scenario", ""), []).append(run)
  return groups


def _worst(values, pick):
  values = [v for v in values if v is not None]
  return pick(values) if values else None


def table_rows(runs):
  rows = []
  for label, members in sorted(group(runs).items()):
    name, town, kph = split_label(label)
    gaps = [r.get("min_gap_m") for r in members]
    rows.append({
      "scenario": name, "town": town, "set_kph": kph, "runs": len(members),
      "passed": sum(1 for r in members if r["passed"]),
      "collisions": sum(1 for r in members if r.get("collision")),
      "min_gap_worst_m": _worst(gaps, min),
      "min_gap_median_m": statistics.median(g for g in gaps if g is not None) if any(g is not None for g in gaps) else None,
      "min_gap_spread_m": (max(g for g in gaps if g is not None) - min(g for g in gaps if g is not None))
                           if sum(g is not None for g in gaps) > 1 else None,
      "approach_kph_median": _worst([r.get("approach_speed_mps") for r in members], lambda v: statistics.median(v) * 3.6),
      "first_lead_gap_median_m": _worst([r.get("first_lead_true_gap_m") for r in members], statistics.median),
      "peak_decel_worst": _worst([r.get("peak_decel_mps2") for r in members], min),
      "false_brake_events": sum(r.get("false_brake_events", 0) for r in members),
      "set_speed_min_kph": _worst([r.get("set_speed_min_kph") for r in members], min),
    })
  return rows


def _cell(value):
  return "-" if value is None else (f"{value:.1f}" if isinstance(value, float) else str(value))


def markdown(rows):
  head = ["scenario", "town", "set_kph", "runs", "passed", "collisions", "min_gap_worst_m", "min_gap_median_m",
          "min_gap_spread_m", "approach_kph_median", "first_lead_gap_median_m", "peak_decel_worst", "false_brake_events", "set_speed_min_kph"]
  lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
  lines += ["| " + " | ".join(_cell(row[h]) for h in head) + " |" for row in rows]
  return "\n".join(lines)


def write_csv(runs, path):
  fields = ["file", "scenario", "passed", "reason", "engaged", "end_reason", "duration_s", "distance_m", "min_gap_m",
            "collision", "collided_with", "impact_speed_mps", "other_contacts", "peak_decel_mps2", "time_below_m1_5_s",
            "false_brake_events", "final_speed_mps", "avg_speed_cruise_mps", "set_speed_final_kph", "set_speed_min_kph",
            "approach_speed_mps", "first_lead_true_gap_m", "steady_gap_m", "steady_gap_range_m", "intended_gap_m", "overtake_max_brake",
            "overtake_yrel_samples", "overtake_yrel_mean_abs_error_m", "realtime_factor"]
  with open(path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    writer.writerows(runs)


def main(argv=None):
  parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  parser.add_argument("directory")
  parser.add_argument("--csv", default=None, help="write every run to this CSV")
  parser.add_argument("--markdown", default=None, help="write the table to this file")
  args = parser.parse_args(argv)
  runs = load_runs(args.directory)
  if not runs:
    print(f"no *.json results in {args.directory}", file=sys.stderr)
    return 1
  table = markdown(table_rows(runs))
  print(table)
  failed = [r for r in runs if not r["passed"]]
  for r in failed:
    print(f"FAIL {r['file']}: {r['reason']}")
  if args.csv:
    write_csv(runs, args.csv)
  if args.markdown:
    pathlib.Path(args.markdown).write_text(table + "\n")
  return 0


if __name__ == "__main__":
  sys.exit(main())
