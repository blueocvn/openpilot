"""Simulation-only low-speed longitudinal comfort policy.

This policy sees only the planner's current request and observed ego state. It
never uses simulator actors, ground truth, or future scene events.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math


PROFILE_LIMITS = {
  "balanced": (1.0, 0.8),
  "gentle": (0.8, 0.6),
  "responsive": (1.2, 1.0),
}
FULL_EFFECT_SPEED_MPS = 40 / 3.6
STOCK_SPEED_MPS = 50 / 3.6


@dataclass(frozen=True)
class TrafficModeConfig:
  mode: bool
  profile: str

  @classmethod
  def from_environment(cls, environment: Mapping[str, str]) -> "TrafficModeConfig":
    mode = environment.get("VN_TRAFFIC_MODE", "0")
    profile = environment.get("VN_TRAFFIC_PROFILE", "balanced")
    if mode not in ("0", "1"):
      raise ValueError("VN_TRAFFIC_MODE must be 0 or 1")
    if profile not in PROFILE_LIMITS:
      raise ValueError(f"VN_TRAFFIC_PROFILE must be one of {', '.join(PROFILE_LIMITS)}")
    return cls(mode == "1", profile)

  def enabled_for(self, *, simulation: bool, longitudinal_active: bool, experimental: bool) -> bool:
    return self.mode and simulation and longitudinal_active and not experimental


class PositiveAccelRamp:
  def __init__(self, profile: str, dt: float):
    if profile not in PROFILE_LIMITS:
      raise ValueError(f"unknown traffic profile: {profile}")
    if not math.isfinite(dt) or dt <= 0:
      raise ValueError("dt must be positive and finite")
    self.accel_limit, self.jerk_limit = PROFILE_LIMITS[profile]
    self.dt = dt
    self.previous_output = 0.0

  def reset(self) -> None:
    self.previous_output = 0.0

  def apply(self, requested_accel: float, *, speed_mps: float, active: bool,
            standstill: bool = False, input_valid: bool = True) -> float:
    if not active or standstill or not input_valid or not math.isfinite(speed_mps) or not math.isfinite(requested_accel):
      self.reset()
      return requested_accel
    if requested_accel <= 0 or speed_mps >= STOCK_SPEED_MPS:
      self.reset()
      return requested_accel

    limited = min(requested_accel, self.accel_limit,
                  max(self.previous_output, 0.0) + self.jerk_limit * self.dt)
    blend = min(1.0, max(0.0, (STOCK_SPEED_MPS - speed_mps) / (STOCK_SPEED_MPS - FULL_EFFECT_SPEED_MPS)))
    output = requested_accel + blend * (limited - requested_accel)
    self.previous_output = output
    return output
