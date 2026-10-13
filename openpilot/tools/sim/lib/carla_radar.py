"""Bounded simulation radar measurements; this module has no CARLA actor access."""

from dataclasses import dataclass
import json
import math

import numpy as np


MAX_RETURNS = 200
MAX_PAYLOAD_BYTES = 32_768
RADAR_MOUNT_X_M = 0.4
RADAR_MOUNT_Z_M = 1.22
RADAR_SENSOR_ATTRIBUTES = {"horizontal_fov": 100, "vertical_fov": 10,
                           "range": 40, "points_per_second": 3000,
                           "sensor_tick": 0.05}


@dataclass(frozen=True)
class RadarFrame:
  frame: int
  timestamp_ns: int
  points: tuple[tuple[float, float, float, float], ...]
  truncated: int = 0
  sim_time_s: float | None = None


def polar_to_vehicle(depth: float, azimuth: float, altitude: float, mount_x_m: float) -> tuple[float, float, float]:
  """CARLA local coordinates: x forward, y right, z up."""
  forward = depth * math.cos(altitude)
  return mount_x_m + forward * math.cos(azimuth), forward * math.sin(azimuth), depth * math.sin(altitude)


def measurement_payload(measurement, *, timestamp_ns: int, mount_x_m: float) -> bytes:
  """CARLA packs each return as float32 [radial velocity, azimuth, altitude, depth]."""
  raw = np.frombuffer(measurement.raw_data, dtype=np.float32)
  if len(raw) % 4:
    raise ValueError("malformed CARLA radar measurement")
  points = []
  for velocity, azimuth, altitude, depth in raw.reshape((-1, 4)):
    if not all(math.isfinite(float(value)) for value in (velocity, azimuth, altitude, depth)) or depth <= 0:
      continue
    x, y, z = polar_to_vehicle(float(depth), float(azimuth), float(altitude), mount_x_m)
    points.append((x, y, z, float(velocity)))
  return encode_radar(int(measurement.frame), timestamp_ns, points,
                      sim_time_s=getattr(measurement, "timestamp", None))


def encode_radar(frame: int, timestamp_ns: int, points: list[tuple[float, float, float, float]],
                 *, sim_time_s: float | None = None) -> bytes:
  if frame < 0 or timestamp_ns <= 0:
    raise ValueError("invalid radar timestamp/frame")
  if sim_time_s is not None and (not math.isfinite(sim_time_s) or sim_time_s < 0):
    raise ValueError("invalid CARLA simulation timestamp")
  if any(len(point) != 4 or not all(math.isfinite(value) for value in point) for point in points):
    raise ValueError("invalid radar point")
  truncated = max(len(points) - MAX_RETURNS, 0)
  selected = sorted(points, key=lambda point: point[0])[:MAX_RETURNS]
  data = {"v": 1, "frame": frame, "timestamp_ns": timestamp_ns,
          "sim_time_s": sim_time_s,
          "points": selected, "truncated": truncated}
  result = json.dumps(data, separators=(",", ":"), allow_nan=False).encode("utf-8")
  if len(result) > MAX_PAYLOAD_BYTES:
    raise ValueError("radar payload too large")
  return result


def decode_radar(payload: bytes) -> RadarFrame | None:
  if len(payload) > MAX_PAYLOAD_BYTES:
    return None
  try:
    data = json.loads(payload)
    if data["v"] != 1 or data["frame"] < 0 or data["timestamp_ns"] <= 0:
      return None
    points = data["points"]
    if len(points) > MAX_RETURNS or data["truncated"] < 0:
      return None
    if any(len(point) != 4 or not all(math.isfinite(value) for value in point) for point in points):
      return None
    sim_time_s = data.get("sim_time_s")
    if sim_time_s is not None and (not isinstance(sim_time_s, (int, float)) or
                                   not math.isfinite(sim_time_s) or sim_time_s < 0):
      return None
    return RadarFrame(int(data["frame"]), int(data["timestamp_ns"]),
                      tuple(tuple(float(value) for value in point) for point in points),
                      int(data["truncated"]), sim_time_s)
  except (KeyError, TypeError, ValueError, OverflowError):
    return None
