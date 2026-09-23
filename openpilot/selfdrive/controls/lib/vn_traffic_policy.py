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
FOLLOW_EXTRA_MAX_S = 0.30
FOLLOW_CAP_S = 2.05
FOLLOW_CLOSING_MIN_MPS = 0.5
FOLLOW_TTC_MAX_S = 5.0
FOLLOW_TTC_FULL_S = 3.0
FOLLOW_RISE_S_PER_S = 0.30
FOLLOW_FALL_S_PER_S = 0.15


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


class TrafficFollowPolicy:
  """Increase MPC time gap for a fresh, already observed closing lead."""

  def __init__(self, dt: float):
    if not math.isfinite(dt) or dt <= 0:
      raise ValueError("dt must be positive and finite")
    self.dt = dt
    self.extra_s = 0.0
    self.diagnostic = self._diagnostic("inactive", None, None, 0.0, None)

  @staticmethod
  def _diagnostic(stage, lead_index, ttc_s, extra_s, t_follow_s):
    return {"stage": stage, "lead_index": lead_index, "ttc_s": ttc_s,
            "extra_s": extra_s, "t_follow_s": t_follow_s}

  def reset(self, stage: str, stock_t_follow: float) -> float:
    self.extra_s = 0.0
    self.diagnostic = self._diagnostic(stage, None, None, 0.0, stock_t_follow)
    return stock_t_follow

  @staticmethod
  def _closing_candidate(lead, speed_mps: float):
    try:
      present = bool(lead.present)
      distance = float(lead.dRel)
      lead_speed = float(lead.vLead)
    except (AttributeError, TypeError, ValueError):
      return None
    closing_speed = speed_mps - lead_speed
    if (not present or not all(math.isfinite(value) for value in (distance, lead_speed, closing_speed)) or
        distance <= 0 or closing_speed <= FOLLOW_CLOSING_MIN_MPS):
      return None
    ttc = distance / closing_speed
    return ttc if math.isfinite(ttc) and ttc < FOLLOW_TTC_MAX_S else None

  def apply(self, stock_t_follow: float, *, speed_mps: float, lead_one, lead_two,
            active: bool, radar_valid: bool) -> float:
    if (not active or not radar_valid or not math.isfinite(speed_mps) or
        not math.isfinite(stock_t_follow)):
      return self.reset("invalid_or_inactive", stock_t_follow)

    candidates = [(index, ttc) for index, lead in enumerate((lead_one, lead_two), start=1)
                  if (ttc := self._closing_candidate(lead, speed_mps)) is not None]
    selected = min(candidates, key=lambda item: item[1]) if candidates else None
    speed_blend = min(1.0, max(0.0, (STOCK_SPEED_MPS - speed_mps) /
                                   (STOCK_SPEED_MPS - FULL_EFFECT_SPEED_MPS)))
    ttc_blend = (min(1.0, max(0.0, (FOLLOW_TTC_MAX_S - selected[1]) /
                                  (FOLLOW_TTC_MAX_S - FOLLOW_TTC_FULL_S))) if selected else 0.0)
    target_extra = min(FOLLOW_EXTRA_MAX_S, max(0.0, FOLLOW_CAP_S - stock_t_follow)) * speed_blend * ttc_blend
    if target_extra > self.extra_s:
      self.extra_s = min(target_extra, self.extra_s + FOLLOW_RISE_S_PER_S * self.dt)
    else:
      self.extra_s = max(target_extra, self.extra_s - FOLLOW_FALL_S_PER_S * self.dt)

    result = min(FOLLOW_CAP_S, stock_t_follow + self.extra_s)
    stage = "closing_lead" if target_extra > 0 else "speed_faded" if selected else "threat_cleared"
    self.diagnostic = self._diagnostic(stage,
                                       selected[0] if selected else None,
                                       selected[1] if selected else None, self.extra_s, result)
    return max(stock_t_follow, result)
