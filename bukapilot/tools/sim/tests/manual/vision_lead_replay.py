#!/usr/bin/env python3
"""Replay recorded model leads through VisionLeadFilter and score each part against the simulator's ground truth.

  PYTHONPATH=<bukapilot root> python3 tools/sim/tests/manual/vision_lead_replay.py <bench dir with *.trace/>

Input: truth.jsonl of every run (bench.sh with TRACE=1): the model's leadsV3[0], the truth, the ego speed.
Output, per scenario and per part combination, over the frames where an object is within 60 m:
  seen      share of frames with a lead
  err_med   median of (dRel - true gap) in metres, over frames with a lead, true gap below 20 m
  err_p90   90th percentile of the same
  v_err     median of (vLead - true object speed) in m/s, same frames
  v_low     share of frames whose vLead is more than 2 m/s BELOW the truth (the risk of phantom braking)
  phantom   share of frames with a lead while nothing is within 60 m (clear-road runs and stretches)
"""
import glob
import json
import os
import statistics
import sys
from types import SimpleNamespace

from selfdrive.controls.lib.radar_helpers import VisionLeadFilter

DT = 0.05
COMBOS = [("off", {}), ("A margin", {"margin": True}), ("B speed", {"speed": True}), ("C hold", {"hold": True}),
          ("A+B+C", {"margin": True, "speed": True, "hold": True})]


def load(directory):
  runs = {}
  for path in sorted(glob.glob(os.path.join(directory, "*.trace", "truth.jsonl"))):
    name = os.path.basename(os.path.dirname(path))[:-len(".trace")]
    rows = [json.loads(line) for line in open(path)]
    runs[name] = [r for r in rows if r.get("engaged") and r.get("model_lead")]
  return runs


def replay(rows, **flags):
  f = VisionLeadFilter(DT, **flags)
  out = []
  for r in rows:
    m = r["model_lead"]
    msg = SimpleNamespace(x=[m["x"]], y=[m["y"]], v=[m["v"]], xStd=[m["x_std"]], yStd=[1.0], vStd=[m["v_std"]], prob=m["prob"])
    out.append(f.update(0, msg, r["ego_speed"]))
  return out


def pct(values, q):
  values = sorted(values)
  return values[min(int(q * len(values)), len(values) - 1)] if values else None


def score(rows, outs):
  near = [(r, o) for r, o in zip(rows, outs) if r["truth_gap"] is not None and r["truth_gap"] < 60]
  seen = [(r, o) for r, o in near if o["status"]]
  close = [(r, o) for r, o in seen if r["truth_gap"] < 20]
  clear = [o for r, o in zip(rows, outs) if r["truth_gap"] is None or r["truth_gap"] >= 60]
  errs = [o["dRel"] - r["truth_gap"] for r, o in close]
  verr = [o["vLead"] - r["truth_v"] for r, o in close]
  return {
    "seen": len(seen) / len(near) if near else None,
    "err_med": statistics.median(errs) if errs else None,
    "err_p90": pct(errs, 0.9),
    "v_err": statistics.median(verr) if verr else None,
    "v_low": sum(1 for v in verr if v < -2.0) / len(verr) if verr else None,
    "phantom": sum(1 for o in clear if o["status"]) / len(clear) if clear else None,
    "n_near": len(near),
  }


def fmt(v):
  return "   -  " if v is None else (f"{v:6.2f}" if isinstance(v, float) else f"{v:6d}")


def main():
  runs = load(sys.argv[1])
  if not runs:
    print("no traces found", file=sys.stderr)
    return 1
  print(f"{'run':28s} {'parts':8s} " + " ".join(f"{h:>6s}" for h in ("seen", "err_med", "err_p90", "v_err", "v_low", "phantom", "n_near")))
  for name, rows in runs.items():
    for label, flags in COMBOS:
      s = score(rows, replay(rows, **flags))
      print(f"{name:28s} {label:8s} " + " ".join(fmt(s[k]) for k in ("seen", "err_med", "err_p90", "v_err", "v_low", "phantom", "n_near")))
  return 0


if __name__ == "__main__":
  sys.exit(main())
