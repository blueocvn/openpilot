"""Open a larger gap behind the lead before a lane change, then let the car accelerate while it crosses.

Opt-in: with ALC_GAP_TF unset (the default) nothing changes. ALC_GAP_TF is the time headway in seconds
used while a blinker is on and the lane change has not started yet.

  idle  -> normal headway
  open  -> blinker on, engaged, lane change not started: headway ramps up to ALC_GAP_TF
  cross -> lane change starting/finishing: headway is held and the in-path lead is ignored
           (long_mpc keeps it when it is closer than the T_FOLLOW_DANGER gap)

After the lane change the headway ramps back to normal.
"""
import os

RAMP_UP = 0.3            # s of headway per second while opening the gap
RAMP_DOWN_TIME = 2.0     # s to ramp back to the normal headway


def read_gap_tf(value=None):
  """Headway in seconds from ALC_GAP_TF, or None when unset or not a positive number."""
  raw = os.environ.get("ALC_GAP_TF") if value is None else value
  try:
    tf = float(raw)
  except (TypeError, ValueError):
    return None
  return tf if tf > 0.0 else None


def phase_for(one_blinker, engaged, crossing):
  if crossing:
    return "cross"
  if one_blinker and engaged:
    return "open"
  return "idle"


class LaneChangeGap:
  def __init__(self, gap_tf=None):
    self.gap_tf = read_gap_tf() if gap_tf is None else gap_tf
    self.tf = None

  @property
  def enabled(self):
    return self.gap_tf is not None

  def update(self, normal_tf, one_blinker, engaged, crossing, dt):
    """Returns (headway to use, ignore the in-path lead)."""
    if not self.enabled:
      return normal_tf, False
    if self.tf is None:
      self.tf = normal_tf

    phase = phase_for(one_blinker, engaged, crossing)
    if phase == "open" and self.gap_tf > normal_tf:
      self.tf = min(self.tf + RAMP_UP * dt, self.gap_tf)
    elif phase == "idle":
      down = max(self.gap_tf - normal_tf, 0.0) / RAMP_DOWN_TIME
      self.tf = max(self.tf - down * dt, normal_tf)
    # cross: hold the current headway
    return self.tf, phase == "cross"
