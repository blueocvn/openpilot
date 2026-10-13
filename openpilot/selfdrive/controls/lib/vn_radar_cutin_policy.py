"""Simulation-only, non-semantic radar cut-in tracking.

Consumes bounded radar measurements and the model path, never CARLA actors or
scene timing. A confirmed track may add a future obstacle to the normal MPC.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from openpilot.tools.sim.lib.carla_radar import RadarFrame


MAX_AGE_NS = 250_000_000
MIN_CONFIRM_SPAN_NS = 100_000_000
MAX_TRACKS = 32
LOOKAHEAD_M = 40.0
LOOKAHEAD_S = 2.0
MAX_CLOSING_TTC_S = 2.5
MAX_ACTION_RANGE_M = 22.0
MAX_ADJACENT_OFFSET_M = 5.2
CORRIDOR_HALF_WIDTH_M = 1.08 + 0.5 + 0.2
# Ego front bumper (~2.4 m from actor origin) plus an assumed 0.8 m object
# half-length. Radar cannot measure the object's full footprint.
CLEARANCE_OFFSET_M = 3.2
COMFORT_BRAKE_MPS2 = 2.5


@dataclass(frozen=True)
class RadarThreat:
  track_id: int
  side: str
  entry_time_s: float
  center_x_m: float
  lateral_m: float
  front_clearance_m: float
  object_speed_mps: float
  lateral_speed_mps: float
  radial_speed_mps: float
  timestamp_ns: int
  offset_history: tuple[tuple[int, float], ...]


@dataclass
class _Track:
  track_id: int
  x: float
  y: float
  offset: float
  radial: float
  timestamp_ns: int
  first_ns: int
  count: int = 1
  lateral_speed: float = 0.0
  offset_history: list[tuple[int, float]] = field(default_factory=list)
  y_history: list[float] = field(default_factory=list)


def _clusters(frame: RadarFrame):
  """Deterministic small clusters; road-height and stationary returns are ignored."""
  points = sorted((p for p in frame.points if 2.0 < p[0] < LOOKAHEAD_M and
                   abs(p[1]) < 10.0 and -0.8 < p[2] < 2.5 and
                   0.8 < abs(p[3]) < 35.0 and all(map(math.isfinite, p))),
                  key=lambda p: (p[0], p[1]))
  groups: list[list[tuple[float, float, float, float]]] = []
  for point in points:
    matching = next((group for group in groups
                     if abs(point[0] - np.median([p[0] for p in group])) < 1.1 and
                     abs(point[1] - np.median([p[1] for p in group])) < 0.8 and
                     abs(point[3] - np.median([p[3] for p in group])) < 1.5), None)
    if matching is None:
      groups.append([point])
    else:
      matching.append(point)
  result = []
  for group in groups:
    if len(group) < 2 or max(p[0] for p in group) - min(p[0] for p in group) > 1.6:
      continue
    result.append(tuple(float(np.median([p[i] for p in group])) for i in (0, 1, 3)))
  return result


class RadarCutInPredictor:
  def __init__(self):
    self.tracks: dict[int, _Track] = {}
    self.next_track_id = 1
    self.last_frame = -1
    self.last_timestamp_ns = 0
    self.threat: RadarThreat | None = None
    self.diagnostic = {"stage": "inactive", "reason": None}

  def _reset(self, reason):
    self.tracks.clear()
    self.threat = None
    self.diagnostic = {"stage": "rejected", "reason": reason}

  @staticmethod
  def _path(model):
    xs = np.asarray(model.position.x, dtype=float)
    ys = np.asarray(model.position.y, dtype=float)
    if len(xs) < 2 or len(xs) != len(ys) or xs[-1] < LOOKAHEAD_M or np.any(np.diff(xs) <= 0):
      raise ValueError("incomplete_path")
    if not np.all(np.isfinite(xs)) or not np.all(np.isfinite(ys)):
      raise ValueError("nonfinite_path")
    return xs, ys

  def update(self, frame: RadarFrame | None, model, *, ego_speed_mps: float,
             now_ns: int, active: bool):
    previous_threat = self.threat
    if not active:
      self._reset("inactive")
      return None
    if frame is None or not isinstance(frame, RadarFrame):
      self._reset("missing_radar")
      return None
    if (frame.frame <= self.last_frame or frame.timestamp_ns <= self.last_timestamp_ns or
        now_ns < frame.timestamp_ns or now_ns - frame.timestamp_ns > MAX_AGE_NS):
      self._reset("stale_or_duplicate_radar")
      return None
    self.last_frame = frame.frame
    self.last_timestamp_ns = frame.timestamp_ns
    if not math.isfinite(ego_speed_mps) or ego_speed_mps < 0 or frame.truncated:
      self._reset("invalid_or_truncated_radar")
      return None
    try:
      path_x, path_y = self._path(model)
    except (AttributeError, TypeError, ValueError) as exc:
      self._reset(str(exc))
      return None

    # Each cluster is associated to at most one historical track. Competing
    # historical tracks deliberately invalidate the match rather than borrowing
    # a nearer object's range.
    assignments = []
    for x, y, radial in _clusters(frame):
      offset = y - float(np.interp(x, path_x, path_y))
      matches = []
      for track in self.tracks.values():
        dt = (frame.timestamp_ns - track.timestamp_ns) / 1e9
        if not 0 < dt <= 0.25:
          continue
        dx = abs(x - (track.x + track.radial * dt))
        dy = abs(offset - (track.offset + track.lateral_speed * dt))
        dv = abs(radial - track.radial)
        if dx < 1.7 and dy < 1.0 and dv < 3.0:
          matches.append((dx + 2.0 * dy + 0.25 * dv, track))
      matches.sort(key=lambda item: item[0])
      if len(matches) > 1 and matches[1][0] - matches[0][0] < 0.5:
        self.tracks.pop(matches[0][1].track_id, None)
        self.tracks.pop(matches[1][1].track_id, None)
        continue
      assignments.append((x, y, offset, radial, matches[0][1] if matches else None))

    used_ids = set()
    confirmed = []
    for x, y, offset, radial, track in assignments:
      if track is None:
        if len(self.tracks) < MAX_TRACKS:
          track_id = self.next_track_id
          self.next_track_id += 1
          self.tracks[track_id] = _Track(track_id, x, y, offset, radial,
                                         frame.timestamp_ns, frame.timestamp_ns,
                                         offset_history=[(frame.timestamp_ns, offset)], y_history=[y])
        continue
      if track.track_id in used_ids:
        self.tracks.pop(track.track_id, None)
        continue
      used_ids.add(track.track_id)
      dt = (frame.timestamp_ns - track.timestamp_ns) / 1e9
      lateral_speed = (offset - track.offset) / dt
      # A one-frame jump cannot establish a dangerous inward motion.
      if abs(lateral_speed) > 12.0:
        self.tracks.pop(track.track_id, None)
        continue
      track.x, track.y, track.offset, track.radial = x, y, offset, radial
      track.timestamp_ns = frame.timestamp_ns
      track.count += 1
      track.lateral_speed = lateral_speed
      track.offset_history.append((frame.timestamp_ns, offset))
      track.offset_history = track.offset_history[-9:]
      track.y_history.append(y)
      track.y_history = track.y_history[-9:]
      if track.count < 5 or frame.timestamp_ns - track.first_ns < 2 * MIN_CONFIRM_SPAN_NS:
        continue
      side = math.copysign(1, offset)
      history = track.offset_history[-5:]
      inward_steps = [side * (older[1] - newer[1])
                      for older, newer in zip(history[:-1], history[1:], strict=True)]
      raw_recent = track.y_history[-5:]
      raw_inward_steps = [side * (older - newer)
                          for older, newer in zip(raw_recent[:-1], raw_recent[1:], strict=True)]
      fast_confirmation = (len(inward_steps) == 4 and min(inward_steps) >= -0.02 and
                           sum(inward_steps) >= 0.35 and len(raw_inward_steps) == 4 and
                           min(raw_inward_steps) >= -0.1 and sum(raw_inward_steps) >= 0.15)
      slow_history = track.offset_history[-9:]
      slow_steps = [side * (older[1] - newer[1])
                    for older, newer in zip(slow_history[:-1], slow_history[1:], strict=True)]
      slow_raw = track.y_history[-9:]
      slow_raw_steps = [side * (older - newer)
                        for older, newer in zip(slow_raw[:-1], slow_raw[1:], strict=True)]
      slow_confirmation = (len(slow_steps) >= 6 and
                           slow_history[-1][0] - slow_history[0][0] >= 300_000_000 and
                           sum(slow_steps) >= 0.55 and sum(slow_raw_steps) >= 0.3 and
                           min(slow_steps) >= -0.25 and min(slow_raw_steps) >= -0.25 and
                           sum(step > 0 for step in slow_steps) >= len(slow_steps) - 3)
      if not (fast_confirmation or slow_confirmation):
        continue
      motion_history = history if fast_confirmation else slow_history
      motion_steps = inward_steps if fast_confirmation else slow_steps
      inward_speed = sum(motion_steps) / ((motion_history[-1][0] - motion_history[0][0]) / 1e9)
      if (abs(offset) <= CORRIDOR_HALF_WIDTH_M or abs(offset) > MAX_ADJACENT_OFFSET_M or
          x > MAX_ACTION_RANGE_M or inward_speed < 0.5):
        continue
      entry_s = (abs(offset) - CORRIDOR_HALF_WIDTH_M) / inward_speed
      if not 0 < entry_s <= LOOKAHEAD_S or x + radial * entry_s <= CLEARANCE_OFFSET_M:
        continue
      front_clearance = max(0.0, x - CLEARANCE_OFFSET_M)
      if radial >= -0.1 or front_clearance / -radial > MAX_CLOSING_TTC_S:
        continue
      object_speed = float(np.clip(ego_speed_mps + radial, 0.0, 30.0))
      confirmed.append(RadarThreat(track.track_id, "right_to_left" if side > 0 else "left_to_right",
                                   entry_s, x, y, front_clearance, object_speed,
                                   lateral_speed, radial, frame.timestamp_ns, tuple(history)))

    self.tracks = {ident: track for ident, track in self.tracks.items()
                   if frame.timestamp_ns - track.timestamp_ns <= MAX_AGE_NS}
    confirmed = [threat for threat in confirmed if threat.track_id in self.tracks]
    if previous_threat is not None and previous_threat.track_id in used_ids:
      track = self.tracks.get(previous_threat.track_id)
      if track is not None:
        side = 1 if previous_threat.side == "right_to_left" else -1
        # Keep an already-confirmed physical track through reflection jitter,
        # but never through a missing frame or a clear movement away from lane.
        outward_jump = side * (track.y - previous_threat.lateral_m)
        clearance = max(0.0, track.x - CLEARANCE_OFFSET_M)
        if (outward_jump <= 0.15 and side * track.y > -0.2 and
            track.x <= MAX_ACTION_RANGE_M and track.radial < -0.1 and
            clearance / -track.radial <= MAX_CLOSING_TTC_S):
          elapsed_s = (frame.timestamp_ns - previous_threat.timestamp_ns) / 1e9
          entry_s = (0.0 if abs(track.offset) <= CORRIDOR_HALF_WIDTH_M else
                     max(0.0, previous_threat.entry_time_s - elapsed_s))
          confirmed.append(RadarThreat(track.track_id, previous_threat.side, entry_s,
                                       track.x, track.y, clearance,
                                       float(np.clip(ego_speed_mps + track.radial, 0.0, 30.0)),
                                       previous_threat.lateral_speed_mps, track.radial,
                                       frame.timestamp_ns, tuple(track.offset_history)))
    if len(confirmed) > 1:
      confirmed.sort(key=lambda threat: threat.front_clearance_m)
    self.threat = confirmed[0] if confirmed else None
    self.diagnostic = ({"stage": "confirmed", "reason": None, "track_id": self.threat.track_id,
                        "side": self.threat.side, "entry_time_s": self.threat.entry_time_s,
                        "distance_m": self.threat.front_clearance_m,
                        "lateral_m": self.threat.lateral_m,
                        "lateral_speed_mps": self.threat.lateral_speed_mps,
                        "radial_speed_mps": self.threat.radial_speed_mps,
                        "offset_history": self.threat.offset_history}
                       if self.threat else {"stage": "tracking", "reason": "no_predicted_conflict",
                                            "tracks": len(self.tracks)})
    return self.threat

  def obstacle(self, horizon_s):
    if self.threat is None:
      return None
    horizon = np.asarray(horizon_s, dtype=float)
    threat = self.threat
    obstacle = (threat.front_clearance_m + threat.object_speed_mps * horizon +
                threat.object_speed_mps**2 / (2 * COMFORT_BRAKE_MPS2))
    obstacle[horizon < threat.entry_time_s] = np.inf
    return obstacle
