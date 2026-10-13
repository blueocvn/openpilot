#!/usr/bin/env python3
"""Observation-only replay of CARLA radar cut-in predictions against scene events."""

import argparse
from bisect import bisect_left
import json
from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.vn_radar_cutin_policy import RadarCutInPredictor
from openpilot.selfdrive.controls.lib.vn_radar_cutin_policy import _clusters
from openpilot.tools.sim.lib.carla_radar import RadarFrame


def rotated_events(directory: Path):
  paths = [directory / "events.jsonl"] + sorted(directory.glob("events-*.jsonl"))
  for path in paths:
    if path.exists():
      with path.open(encoding="utf-8") as stream:
        for line in stream:
          yield json.loads(line)


def planner_models(report_dir: Path):
  for event in rotated_events(report_dir / "planner"):
    if event.get("type") != "planner_input":
      continue
    data = event["data"]
    position = data["modelV2"]["position"]
    yield (event["tick_time_ns"],
           SimpleNamespace(position=SimpleNamespace(x=position["x"], y=position["y"])),
           float(data["carState"]["vEgo"]))


def analyze(report_dir: Path, *, debug=False, debug_start_frame=None):
  predictor = RadarCutInPredictor()
  models = iter(planner_models(report_dir))
  next_model = next(models, None)
  current_model = None
  predictions = []
  track_debug = []
  first_track_id = None
  last_track_xy = None
  frames = 0
  for event in rotated_events(report_dir / "radar"):
    if event.get("type") != "radar_frame":
      continue
    timestamp_ns = event["timestamp_ns"]
    while next_model is not None and next_model[0] <= timestamp_ns:
      current_model = next_model
      next_model = next(models, None)
    if current_model is None or timestamp_ns - current_model[0] > 250_000_000:
      continue
    frame = RadarFrame(event["frame"], timestamp_ns,
                       tuple(tuple(point) for point in event["points"]), event["truncated"],
                       event.get("sim_time_s"))
    predictor.update(frame, current_model[1], ego_speed_mps=current_model[2],
                     now_ns=timestamp_ns, active=True)
    frames += 1
    if predictor.threat is not None:
      predictions.append({"timestamp_ns": timestamp_ns, **predictor.diagnostic})
      if first_track_id is None:
        first_track_id = predictor.threat.track_id
    debug_active = (debug and (first_track_id is not None or debug_start_frame is not None) and
                    len(track_debug) < 30 and (debug_start_frame is None or frame.frame >= debug_start_frame))
    if debug_active:
      track = predictor.tracks.get(first_track_id)
      if track is not None:
        last_track_xy = (track.x, track.y)
      near_clusters = ([cluster for cluster in _clusters(frame)
                        if last_track_xy is not None and
                        abs(cluster[0] - last_track_xy[0]) < 3 and
                        abs(cluster[1] - last_track_xy[1]) < 2])
      track_debug.append({"timestamp_ns": timestamp_ns, "radar_frame": frame.frame,
                          "threat": predictor.threat is not None,
                          "candidate_tracks": [(t.track_id, round(t.x, 2), round(t.y, 2), round(t.offset, 2),
                                                 t.count, round(t.radial, 2)) for t in predictor.tracks.values()
                                                if 4 < t.x < 16 and 1 < t.y < 5],
                          "near_clusters": near_clusters,
                          "track": None if track is None else
                          {"x": track.x, "y": track.y, "offset": track.offset,
                           "count": track.count, "radial": track.radial}})
  scene_events = []
  poses = []
  for event in rotated_events(report_dir):
    if event.get("type") in ("cut_in_started", "cut_in_entered", "cut_in_completed", "collision"):
      scene_events.append({key: event.get(key) for key in
                           ("type", "scene_time_s", "host_monotonic_ns", "actor_id", "direction")})
    elif event.get("type") == "control_sample":
      actor = next((actor for actor in event["actors"] if actor["role"] == "cut_in"), None)
      poses.append((event["host_monotonic_ns"], actor))
  pose_times = [timestamp for timestamp, _ in poses]
  matched = []
  unmatched = []
  for prediction in predictions:
    index = bisect_left(pose_times, prediction["timestamp_ns"])
    choices = poses[max(0, index - 1):index + 1]
    timestamp, actor = min(choices, key=lambda item: abs(item[0] - prediction["timestamp_ns"]))
    if (actor is not None and abs(timestamp - prediction["timestamp_ns"]) < 100_000_000 and
        abs(actor["ego_forward_gap_m"] - prediction["distance_m"]) < 2.0 and
        abs(actor["ego_lateral_offset_m"] - prediction["lateral_m"]) < 1.0):
      matched.append(prediction)
    else:
      unmatched.append(prediction)
  cut_starts = [event for event in scene_events if event["type"] == "cut_in_started"]
  cut_in_matches = []
  for index, event in enumerate(cut_starts):
    end_ns = (cut_starts[index + 1]["host_monotonic_ns"] if index + 1 < len(cut_starts)
              else float("inf"))
    window = [prediction for prediction in matched
              if event["host_monotonic_ns"] <= prediction["timestamp_ns"] < end_ns]
    cut_in_matches.append({"actor_id": event["actor_id"], "direction": event["direction"],
                           "confirmed_frames": len(window),
                           "first_confirmed_ns": window[0]["timestamp_ns"] if window else None,
                           "last_confirmed_ns": window[-1]["timestamp_ns"] if window else None})
  return {"frames_with_model": frames, "confirmed_frames": len(predictions),
          "matched_frames": len(matched), "unmatched_frames": len(unmatched),
          "unmatched_before_cut_in": sum(p["timestamp_ns"] < next((e["host_monotonic_ns"] for e in scene_events
                                                                   if e["type"] == "cut_in_started"), 0)
                                         for p in unmatched),
          "unmatched_examples": unmatched[:5],
          "first_confirmed": predictions[0] if predictions else None,
          "first_matched": matched[0] if matched else None,
          "last_matched": matched[-1] if matched else None,
          "cut_in_matches": cut_in_matches,
          "last_confirmed": predictions[-1] if predictions else None,
          **({"track_debug": track_debug} if debug else {}),
          "scene_events": scene_events}


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("report_dir", type=Path)
  parser.add_argument("--debug", action="store_true")
  args = parser.parse_args()
  print(json.dumps(analyze(args.report_dir, debug=args.debug), indent=2))
