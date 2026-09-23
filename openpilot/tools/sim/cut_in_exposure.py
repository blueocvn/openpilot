"""Bounded, measured CARLA cut-in exposure records.

This module has no CARLA dependency so reports can be scored offline.
"""

import math


class ExposureCollector:
  """Retain only active cut-ins while emitting completed evidence records."""
  def __init__(self):
    self._active = {}
    self._finished = []

  @property
  def active_count(self):
    return len(self._active)

  def add(self, record):
    kind = record.get("type")
    actor_id = record.get("actor_id")
    if kind == "cut_in_started" and actor_id is not None:
      self._active[actor_id] = {"actor_id": actor_id, "direction": record.get("direction"),
                                "entry_time_s": None, "exit_time_s": None, "entry_gap_m": None, "hold_s": None,
                                "ego_speed_at_entry_mps": None, "bike_speed_at_entry_mps": None,
                                "contact": False, "complete": False, "evidence_missing": False,
                                "pose_sample_count": 0, "invalid_pose_sample_count": 0,
                                "spawn_footprints_clear": record.get("spawn_footprints_clear") is True,
                                "front_of_ego_teleport": record.get("front_of_ego_teleport") is True}
    elif kind == "collision":
      other_id = record.get("other_actor_id")
      if other_id in self._active:
        self._active[other_id]["contact"] = True
    elif kind == "control_sample":
      self._add_control_sample(record)
    elif actor_id in self._active:
      event = self._active[actor_id]
      if kind == "cut_in_entered_ego_lane":
        event["entry_time_s"] = record.get("scene_time_s")
        for source, target in (("actual_gap_m", "entry_gap_m"), ("ego_speed_mps", "ego_speed_at_entry_mps"),
                               ("bike_speed_mps", "bike_speed_at_entry_mps")):
          event[target] = record.get(source)
        if not all(_finite(event[key]) for key in ("entry_time_s", "entry_gap_m", "ego_speed_at_entry_mps", "bike_speed_at_entry_mps")):
          event["evidence_missing"] = True
      elif kind == "cut_in_left_ego_lane":
        event["exit_time_s"] = record.get("scene_time_s")
        event["hold_s"] = record.get("actual_hold_s")
        if not _finite(event["hold_s"]):
          event["evidence_missing"] = True
      elif kind == "cut_in_finished":
        event["complete"] = True
        if (record.get("direction") != event["direction"] or event["entry_time_s"] is None or event["hold_s"] is None or
            event["invalid_pose_sample_count"] or not event["pose_sample_count"] or not event["spawn_footprints_clear"] or
            event["front_of_ego_teleport"]):
          event["evidence_missing"] = True
        self._finished.append(event)
        del self._active[actor_id]

  def finish(self):
    for event in self._active.values():
      event["evidence_missing"] = True
      self._finished.append(event)
    self._active.clear()
    return list(self._finished)

  def _add_control_sample(self, record):
    sample_ns = record.get("host_monotonic_ns")
    openpilot = record.get("openpilot", {})
    snapshot_ns = openpilot.get("snapshot_monotonic_ns")
    statuses = openpilot.get("message_status", {})
    relevant = (statuses.get("modelV2"), statuses.get("radarState"))
    source_ok = all(isinstance(status, dict) and status.get("valid") is True and status.get("alive") is True and
                    isinstance(status.get("mono_time_ns"), int) and 0 <= sample_ns - status["mono_time_ns"] <= 250_000_000
                    for status in relevant) if isinstance(sample_ns, int) else False
    freshness_ok = (isinstance(sample_ns, int) and isinstance(snapshot_ns, int) and
                    0 <= sample_ns - snapshot_ns <= 250_000_000 and source_ok)
    actors = {actor.get("id"): actor for actor in record.get("actors", []) if actor.get("id") is not None}
    for actor_id, event in self._active.items():
      actor = actors.get(actor_id)
      event["pose_sample_count"] += 1
      pose_ok = actor is not None and all(_finite(actor.get(key)) for key in
                                          ("ego_forward_gap_m", "ego_lateral_offset_m", "bumper_gap_m"))
      if not freshness_ok or not pose_ok:
        event["invalid_pose_sample_count"] += 1
        event["evidence_missing"] = True


def _finite(value):
  return isinstance(value, (int, float)) and math.isfinite(value)
