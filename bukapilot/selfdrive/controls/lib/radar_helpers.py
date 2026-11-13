from common.numpy_fast import clip, mean
from common.kalman.simple_kalman import KF1D
from selfdrive.config import RADAR_TO_CAMERA


# the longer lead decels, the more likely it will keep decelerating
# TODO is this a good default?
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1   # Kalman filter states enum

# stationary qualification parameters
v_ego_stationary = 4.   # no stationary object flag below this speed


# Vision-only lead refinements (cars without radar points). Each part has its own flag, off by default.
LEAD_PROB_MIN = 0.5            # the model's lead is used above this probability
LEAD_MARGIN_K_STD = 1.0        # distance margin = K * xStd, between the two limits (m)
LEAD_MARGIN_MIN = 3.0
LEAD_MARGIN_MAX = 10.0
LEAD_SPEED_XSTD_MAX = 5.0      # the speed from distance trend is only trusted while the distance is this certain (m)
LEAD_SPEED_FILTER_S = 0.5      # low-pass window of the distance trend
LEAD_SPEED_WARMUP = 3          # frames of trend needed before it may lower the model's speed
LEAD_HOLD_S = 1.0              # keep a lead this long after its probability drops below LEAD_PROB_MIN
LEAD_HOLD_RELEASE_PROB = 0.1   # ... but let go at once below this


class _LeadSlot():
  def __init__(self):
    self.reset()

  def reset(self):
    self.last = None        # (dRel, yRel, vLead, vRel at the last good frame)
    self.age = 0            # frames since the last good frame
    self.prev_d = None      # raw distance of the previous good frame, for the trend
    self.rate = 0.0         # filtered d(dRel)/dt
    self.rate_n = 0


class VisionLeadFilter():
  """Makes the camera model's lead safer to follow. One instance per RadarD; `slot` is the lead index (0 or 1).

  margin: dRel is shortened by K * xStd (limits above), because the model over-estimates close distances.
  speed:  vLead is capped by the speed the distance trend implies (v_ego + d(dRel)/dt), so a stationary object the
          model reports as moving is still seen as stationary.
  hold:   the lead is kept for LEAD_HOLD_S after its probability dips below LEAD_PROB_MIN, extrapolated with its last
          relative speed; it is released at once when the probability falls below LEAD_HOLD_RELEASE_PROB.
  """

  def __init__(self, dt, margin=False, speed=False, hold=False):
    self.dt = dt
    self.margin = margin
    self.speed = speed
    self.hold = hold
    self.slots = {}

  def update(self, slot, lead_msg, v_ego):
    """RadarState lead dict for this slot, {'status': False} when there is none."""
    st = self.slots.setdefault(slot, _LeadSlot())
    prob = float(lead_msg.prob)
    if prob > LEAD_PROB_MIN:
      return self._seen(st, lead_msg, v_ego, prob)
    if self.hold and st.last is not None and prob >= LEAD_HOLD_RELEASE_PROB and (st.age + 1) * self.dt <= LEAD_HOLD_S:
      st.age += 1
      d, y, v_lead, _ = st.last
      v_rel = v_lead - v_ego
      return self._lead_dict(max(d + v_rel * st.age * self.dt, 0.0), y, v_lead, v_ego, prob)
    st.reset()
    return {'status': False}

  def _seen(self, st, lead_msg, v_ego, prob):
    d = float(lead_msg.x[0]) - RADAR_TO_CAMERA
    y = -float(lead_msg.y[0])
    v_lead = float(lead_msg.v[0])
    x_std = float(lead_msg.xStd[0])

    if self.speed:
      if st.prev_d is not None and x_std < LEAD_SPEED_XSTD_MAX:
        raw = (d - st.prev_d) / self.dt
        alpha = self.dt / (LEAD_SPEED_FILTER_S + self.dt)
        st.rate = raw if st.rate_n == 0 else st.rate + alpha * (raw - st.rate)   # start from the first sample, not 0
        st.rate_n += 1
        if st.rate_n >= LEAD_SPEED_WARMUP:
          v_lead = min(v_lead, max(v_ego + st.rate, 0.0))
      else:
        st.rate, st.rate_n = 0.0, 0
      st.prev_d = d

    if self.margin:
      d = max(d - clip(LEAD_MARGIN_K_STD * x_std, LEAD_MARGIN_MIN, LEAD_MARGIN_MAX), 0.0)

    st.last = (d, y, v_lead, v_lead - v_ego)
    st.age = 0
    return self._lead_dict(d, y, v_lead, v_ego, prob)

  @staticmethod
  def _lead_dict(d, y, v_lead, v_ego, prob):
    return {
      "dRel": float(d),
      "yRel": float(y),
      "vRel": float(v_lead - v_ego),
      "vLead": float(v_lead),
      "vLeadK": float(v_lead),
      "aLeadK": 0.0,
      "aLeadTau": 0.3,
      "fcw": False,
      "modelProb": prob,
      "radar": False,
      "status": True
    }


class Track():
  def __init__(self, v_lead, kalman_params):
    self.cnt = 0
    self.aLeadTau = _LEAD_ACCEL_TAU
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel, y_rel, v_rel, v_lead, measured):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead
    self.measured = measured   # measured or estimate

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau *= 0.9

    self.cnt += 1

  def get_key_for_cluster(self):
    # Weigh y higher since radar is inaccurate in this dimension
    return [self.dRel, self.yRel*2, self.vRel]

  def reset_a_lead(self, aLeadK, aLeadTau):
    self.kf = KF1D([[self.vLead], [aLeadK]], self.K_A, self.K_C, self.K_K)
    self.aLeadK = aLeadK
    self.aLeadTau = aLeadTau


class Cluster():
  def __init__(self):
    self.tracks = set()

  def add(self, t):
    # add the first track
    self.tracks.add(t)

  # TODO: make generic
  @property
  def dRel(self):
    return mean([t.dRel for t in self.tracks])

  @property
  def yRel(self):
    return mean([t.yRel for t in self.tracks])

  @property
  def vRel(self):
    return mean([t.vRel for t in self.tracks])

  @property
  def aRel(self):
    return mean([t.aRel for t in self.tracks])

  @property
  def vLead(self):
    return mean([t.vLead for t in self.tracks])

  @property
  def dPath(self):
    return mean([t.dPath for t in self.tracks])

  @property
  def vLat(self):
    return mean([t.vLat for t in self.tracks])

  @property
  def vLeadK(self):
    return mean([t.vLeadK for t in self.tracks])

  @property
  def aLeadK(self):
    if all(t.cnt <= 1 for t in self.tracks):
      return 0.
    else:
      return mean([t.aLeadK for t in self.tracks if t.cnt > 1])

  @property
  def aLeadTau(self):
    if all(t.cnt <= 1 for t in self.tracks):
      return _LEAD_ACCEL_TAU
    else:
      return mean([t.aLeadTau for t in self.tracks if t.cnt > 1])

  @property
  def measured(self):
    return any(t.measured for t in self.tracks)

  def get_RadarState(self, model_prob=0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "aLeadTau": float(self.aLeadTau)
    }

  def get_RadarState_from_vision(self, lead_msg, v_ego):
    return {
      "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
      "yRel": float(-lead_msg.y[0]),
      "vRel": float(lead_msg.v[0] - v_ego),
      "vLead": float(lead_msg.v[0]),
      "vLeadK": float(lead_msg.v[0]),
      "aLeadK": 0.0,
      "aLeadTau": 0.3,
      "fcw": False,
      "modelProb": float(lead_msg.prob),
      "radar": False,
      "status": True
    }

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret

  def potential_low_speed_lead(self, v_ego):
    # stop for stuff in front of you and low speed, even without model confirmation
    return abs(self.yRel) < 1.5 and (v_ego < v_ego_stationary) and self.dRel < 25

  def is_potential_fcw(self, model_prob):
    return model_prob > .9
