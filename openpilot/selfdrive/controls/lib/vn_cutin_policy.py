"""Simulation-only cut-in prediction from modelV2; never consumes CARLA actor state."""

from dataclasses import dataclass
import math

import numpy as np


LOOKAHEAD_M = 40.0
PREDICTION_S = 2.0
# The CARLA Tesla body is 2.16 m wide; 0.5 m is an assumed motorcycle half-width.
CORRIDOR_HALF_WIDTH_M = 1.08 + 0.5 + 0.2
# Cross-frame motion has three independent observations, so allow uncertainty
# up to the 1.8 m conflict corridor while keeping trajectory-only entry at 1.5 m.
FRAME_MOTION_MAX_Y_STD_M = 1.8
MIN_PROBABILITY = 0.55
MAX_MODEL_AGE_NS = 250_000_000
CONFIRM_FRAMES = 3
CONFIRM_SPAN_NS = 100_000_000
RETRACT_NS = 300_000_000
COMFORT_BRAKE_MPS2 = 2.5
# Match radard.get_RadarState_from_vision: model lead x is camera-relative,
# whereas MPC radar obstacles use dRel after this longitudinal correction.
MODEL_X_TO_RADAR_M = 1.52


@dataclass(frozen=True)
class CutInThreat:
  side: str
  hypothesis_index: int
  probability: float
  prob_time_s: float
  entry_time_s: float
  center_x_m: float
  distance_used_m: float
  x_std_m: float
  y_std_at_entry_m: float
  times_s: np.ndarray
  xs_m: np.ndarray
  speeds_mps: np.ndarray
  release_fraction: float = 0.0


@dataclass(frozen=True)
class _LeadObservation:
  hypothesis_index: int
  probability: float
  prob_time_s: float
  lateral_m: float
  x_m: float
  speed_mps: float
  x_std_m: float
  y_std_m: float
  longitudinal_reliable: bool
  times_s: np.ndarray
  model_xs_m: np.ndarray
  xs_m: np.ndarray
  offsets_m: np.ndarray
  y_stds_m: np.ndarray
  speeds_mps: np.ndarray


@dataclass(frozen=True)
class _CutInCandidate:
  observation: _LeadObservation
  entry_time_s: float
  y_std_at_entry_m: float
  evidence: str
  actionable: bool = True


class CutInPredictor:
  def __init__(self):
    self.last_model_time_ns = None
    self.last_track_time_ns = None
    self.first_confirmation_ns = None
    self.confirmed_at_ns = None
    self.frame_count = 0
    self.previous = None
    self.last_track_time_ns = None
    self.threat = None
    self.release_at_ns = None
    self.evidence = None
    self.diagnostic = {"stage": "inactive", "reason": None}

  def _reset(self, reason):
    self.first_confirmation_ns = None
    self.confirmed_at_ns = None
    self.frame_count = 0
    self.previous = None
    self.threat = None
    self.release_at_ns = None
    self.evidence = None
    self.diagnostic = {"stage": "rejected", "reason": reason}
    return None

  @staticmethod
  def _observation(model, lead, hypothesis_index):
    probability, prob_time = float(lead.prob), float(lead.probTime)
    if not math.isfinite(probability) or not math.isfinite(prob_time):
      raise ValueError("nonfinite_probability")
    if probability < MIN_PROBABILITY or prob_time > PREDICTION_S:
      return None
    path_x = np.asarray(model.position.x, dtype=float)
    path_y = np.asarray(model.position.y, dtype=float)
    times = np.asarray(lead.t, dtype=float)
    model_xs = np.asarray(lead.x, dtype=float)
    ys = np.asarray(lead.y, dtype=float)
    speeds = np.asarray(lead.v, dtype=float)
    x_std = np.asarray(lead.xStd, dtype=float)
    y_std = np.asarray(lead.yStd, dtype=float)
    if (min(len(path_x), len(path_y), len(times), len(model_xs), len(ys), len(speeds), len(x_std), len(y_std)) < 2 or
        len({len(times), len(model_xs), len(ys), len(speeds), len(x_std), len(y_std)}) != 1 or
        len(path_x) != len(path_y)):
      raise ValueError("incomplete_trajectory")
    if not all(np.all(np.isfinite(a)) for a in (path_x, path_y, times, model_xs, ys, speeds, x_std, y_std)):
      raise ValueError("nonfinite_trajectory")
    xs = model_xs - MODEL_X_TO_RADAR_M
    if (np.any(np.diff(path_x) <= 0) or np.any(np.diff(times) <= 0) or
        times[0] > 0 or times[-1] < PREDICTION_S or xs[0] <= 0 or xs[0] > LOOKAHEAD_M or
        x_std[0] < 0 or np.any(y_std < 0) or np.any(speeds < 0)):
      return None
    if path_x[-1] < max(model_xs[times <= PREDICTION_S], default=model_xs[0]):
      raise ValueError("short_ego_path")
    offsets = ys - np.interp(model_xs, path_x, path_y)
    return _LeadObservation(hypothesis_index, probability, prob_time, float(offsets[0]),
                            float(xs[0]), float(speeds[0]), float(x_std[0]), float(y_std[0]),
                            bool(x_std[0] <= 0.75 * xs[0]),
                            times, model_xs, xs, offsets, y_std, speeds)

  @staticmethod
  def _trajectory_candidate(model, observation):
    if not observation.longitudinal_reliable:
      raise ValueError("longitudinal_uncertainty")
    lateral_now = observation.lateral_m
    if abs(lateral_now) <= CORRIDOR_HALF_WIDTH_M:
      return None
    times = observation.times_s
    model_xs = observation.model_xs_m
    offsets = observation.offsets_m
    y_std = observation.y_stds_m
    path_x = np.asarray(model.position.x, dtype=float)
    sample_times = np.unique(np.r_[times[times <= PREDICTION_S], PREDICTION_S])
    sample_offsets = np.interp(sample_times, times, offsets)
    side = math.copysign(1.0, lateral_now)
    if side * (sample_offsets[1] - lateral_now) >= -0.3:
      return None
    entry_time = None
    for t0, t1, y0, y1 in zip(sample_times[:-1], sample_times[1:], sample_offsets[:-1], sample_offsets[1:], strict=True):
      if side * y0 > CORRIDOR_HALF_WIDTH_M and side * y1 <= CORRIDOR_HALF_WIDTH_M:
        fraction = (side * y0 - CORRIDOR_HALF_WIDTH_M) / max(side * (y0 - y1), 1e-6)
        entry_time = float(t0 + np.clip(fraction, 0., 1.) * (t1 - t0))
        break
    if entry_time is None:
      return None
    entry_x = float(np.interp(entry_time, times, model_xs))
    ego_x = float(np.interp(entry_time, model.position.t, path_x))
    entry_y_std = float(np.interp(entry_time, times, y_std))
    if entry_x <= ego_x or entry_x <= MODEL_X_TO_RADAR_M or entry_y_std > 1.5:
      raise ValueError("uncertain_or_behind_at_entry")
    return _CutInCandidate(observation, entry_time, entry_y_std, "model_trajectory")

  @staticmethod
  def _motion_candidate(previous, current, model_time_ns, previous_time_ns):
    delta_s = (model_time_ns - previous_time_ns) / 1e9
    if delta_s <= 0 or previous.lateral_m * current.lateral_m < 0:
      return None
    side = math.copysign(1.0, previous.lateral_m)
    lateral_closing_speed = -side * (current.lateral_m - previous.lateral_m) / delta_s
    if lateral_closing_speed < 0.3:
      return None
    entry_time = max(0.0, (abs(current.lateral_m) - CORRIDOR_HALF_WIDTH_M) / lateral_closing_speed)
    if entry_time > PREDICTION_S:
      return None
    if float(np.interp(entry_time, current.times_s, current.xs_m)) <= 0:
      return None
    actionable = current.longitudinal_reliable and current.y_std_m <= FRAME_MOTION_MAX_Y_STD_M
    return _CutInCandidate(current, entry_time, current.y_std_m, "frame_motion", actionable)

  @staticmethod
  def _match_observation(previous, observations, model_time_ns, previous_time_ns, ego_speed_mps):
    delta_s = (model_time_ns - previous_time_ns) / 1e9
    if delta_s <= 0:
      return None
    expected_x = previous.x_m + (previous.speed_mps - ego_speed_mps) * delta_s
    same_horizon = [observation for observation in observations
                    if abs(observation.prob_time_s - previous.prob_time_s) < 0.1]
    matches = []
    for observation in same_horizon or observations:
      dx = abs(observation.x_m - expected_x)
      dy = abs(observation.lateral_m - previous.lateral_m)
      dv = abs(observation.speed_mps - previous.speed_mps)
      if dx <= 4.0 and dy <= 2.0 and dv <= 5.0:
        matches.append((dx + 2.0 * dy + 0.5 * dv, observation))
    matches.sort(key=lambda item: (item[0], -item[1].probability))
    if len(matches) > 1 and matches[1][0] - matches[0][0] < 0.5:
      first, second = matches[:2]
      if (abs(first[1].x_m - second[1].x_m) > 2.0 or
          first[1].lateral_m * second[1].lateral_m < 0):
        raise ValueError("ambiguous_frame_association")
    return matches[0][1] if matches else None

  def _release(self, reason, now_ns, observation=None, model_time_ns=None):
    if self.threat is not None:
      if observation is None:
        self.previous = None
        self.last_track_time_ns = None
      else:
        self.previous = observation
        self.last_track_time_ns = model_time_ns
      self.release_at_ns = self.release_at_ns or now_ns
      fraction = (now_ns - self.release_at_ns) / RETRACT_NS
      if fraction < 1:
        self.threat = CutInThreat(**{**self.threat.__dict__, "release_fraction": fraction})
        self.diagnostic = self._describe("retracting", self.threat, "release")
        return self.threat
    return self._reset(reason)

  def _start_track(self, observation, model_time_ns, candidate=None):
    self.previous = observation
    self.last_track_time_ns = model_time_ns
    self.first_confirmation_ns = model_time_ns
    self.frame_count = 1
    self.confirmed_at_ns = None
    self.threat = None
    self.release_at_ns = None
    self.evidence = candidate.evidence if candidate is not None else None
    self.diagnostic = {"stage": "confirming" if candidate is not None else "tracking",
                       "reason": None, "frames": 1,
                       "hypothesis_index": observation.hypothesis_index,
                       "probability": observation.probability,
                       "evidence": self.evidence}
    return None

  def update(self, model, *, ego_speed_mps, model_time_ns, now_ns, fresh, active):
    if not active:
      return self._reset("inactive")
    if (not math.isfinite(ego_speed_mps) or model_time_ns <= 0 or
        now_ns < model_time_ns or now_ns - model_time_ns > MAX_MODEL_AGE_NS):
      return self._reset("stale_or_invalid_model")
    if self.last_model_time_ns is not None and model_time_ns < self.last_model_time_ns:
      self.last_model_time_ns = model_time_ns
      return self._reset("nonmonotonic_model_time")
    if not fresh:
      return self._reset("missing_model_message")
    if model_time_ns == self.last_model_time_ns:
      return self._reset("nonprogressing_model_time")
    self.last_model_time_ns = model_time_ns

    observations = []
    observation_errors = []
    try:
      leads = model.leadsV3
    except AttributeError as error:
      return self._reset(str(error))
    for hypothesis_index, lead in enumerate(leads):
      try:
        observation = self._observation(model, lead, hypothesis_index)
      except (AttributeError, TypeError, ValueError, IndexError) as error:
        observation_errors.append(str(error))
        continue
      if observation is not None:
        observations.append(observation)

    candidates = []
    candidate_errors = {}
    for observation in observations:
      try:
        candidate = self._trajectory_candidate(model, observation)
      except (AttributeError, TypeError, ValueError, IndexError) as error:
        candidate_errors[observation.hypothesis_index] = str(error)
        continue
      if candidate is not None:
        candidates.append(candidate)
    candidates.sort(key=lambda item: item.observation.probability, reverse=True)
    if len(candidates) > 1 and candidates[1].observation.probability >= candidates[0].observation.probability - 0.15:
      first, second = candidates[:2]
      if (abs(first.observation.x_m - second.observation.x_m) > 4.0 or
          first.observation.lateral_m * second.observation.lateral_m < 0):
        return self._reset("ambiguous_hypotheses")

    if not observations:
      return self._release(observation_errors[0] if observation_errors else "no_predicted_conflict", now_ns)

    if self.previous is None:
      if candidates:
        return self._start_track(candidates[0].observation, model_time_ns, candidates[0])
      plausible = [observation for observation in observations
                   if abs(observation.lateral_m) > CORRIDOR_HALF_WIDTH_M]
      if plausible:
        plausible.sort(key=lambda item: item.probability, reverse=True)
        return self._start_track(plausible[0], model_time_ns)
      return self._release("no_predicted_conflict", now_ns)

    previous = self.previous
    previous_time_ns = self.first_confirmation_ns if self.frame_count == 1 else self.last_track_time_ns
    try:
      observation = self._match_observation(previous, observations, model_time_ns,
                                            previous_time_ns, ego_speed_mps)
    except ValueError as error:
      return self._reset(str(error))
    if observation is None:
      if candidates:
        return self._start_track(candidates[0].observation, model_time_ns, candidates[0])
      return self._release("track_lost", now_ns)

    candidate = next((item for item in candidates if item.observation is observation), None)
    if candidate is None:
      try:
        candidate = self._motion_candidate(previous, observation, model_time_ns, previous_time_ns)
      except ValueError as error:
        return self._reset(str(error))
    if candidate is None and self.threat is not None and abs(observation.lateral_m) <= CORRIDOR_HALF_WIDTH_M:
      candidate = _CutInCandidate(observation, 0.0, observation.y_std_m, "observed_overlap")
    if candidate is None:
      if observation.hypothesis_index in candidate_errors:
        if self.threat is not None:
          return self._release(candidate_errors[observation.hypothesis_index], now_ns,
                               observation, model_time_ns)
        return self._reset(candidate_errors[observation.hypothesis_index])
      if candidates:
        return self._start_track(candidates[0].observation, model_time_ns, candidates[0])
      if self.threat is not None:
        return self._release("no_predicted_conflict", now_ns, observation, model_time_ns)
      return self._start_track(observation, model_time_ns)

    self.last_track_time_ns = model_time_ns
    self.release_at_ns = None
    self.frame_count += 1
    self.previous = observation
    self.evidence = candidate.evidence
    if not candidate.actionable:
      if self.threat is not None:
        return self._release("motion_uncertainty", now_ns, observation, model_time_ns)
      self.diagnostic = {"stage": "tracking", "reason": "motion_uncertainty",
                         "frames": self.frame_count,
                         "probability": observation.probability,
                         "entry_time_s": candidate.entry_time_s,
                         "evidence": candidate.evidence}
      return None
    if self.frame_count < CONFIRM_FRAMES or model_time_ns - self.first_confirmation_ns < CONFIRM_SPAN_NS:
      self.diagnostic = {"stage": "confirming", "reason": None, "frames": self.frame_count,
                         "probability": observation.probability,
                         "entry_time_s": candidate.entry_time_s, "evidence": candidate.evidence}
      return None

    if self.confirmed_at_ns is None:
      self.confirmed_at_ns = self.first_confirmation_ns
    stable_fraction = np.clip((model_time_ns - self.confirmed_at_ns) / CONFIRM_SPAN_NS, 0., 1.)
    approach_fraction = np.clip((PREDICTION_S - candidate.entry_time_s) / 1.0, 0., 1.)
    correction = min(observation.x_std_m, 0.5 * observation.x_m) * stable_fraction * approach_fraction
    side_lateral = previous.lateral_m if abs(observation.lateral_m) < 1e-3 else observation.lateral_m
    self.threat = CutInThreat("right_to_left" if side_lateral > 0 else "left_to_right",
                             observation.hypothesis_index, observation.probability,
                             observation.prob_time_s, candidate.entry_time_s,
                             observation.x_m, observation.x_m - correction,
                             observation.x_std_m, candidate.y_std_at_entry_m,
                             observation.times_s, observation.xs_m, observation.speeds_mps)
    self.diagnostic = self._describe("confirmed", self.threat, candidate.evidence)
    return self.threat

  @staticmethod
  def _describe(stage, threat, evidence=None):
    return {"stage": stage, "reason": None, "hypothesis_index": threat.hypothesis_index,
            "probability": threat.probability,
            "prob_time_s": threat.prob_time_s, "side": threat.side,
            "entry_time_s": threat.entry_time_s, "center_x_m": threat.center_x_m,
            "distance_used_m": threat.distance_used_m, "x_std_m": threat.x_std_m,
            "y_std_at_entry_m": threat.y_std_at_entry_m,
            "release_fraction": threat.release_fraction, "evidence": evidence}

  def obstacle(self, horizon_s):
    if self.threat is None:
      return None
    threat = self.threat
    horizon = np.asarray(horizon_s, dtype=float)
    xs = np.interp(horizon, threat.times_s, threat.xs_m)
    speeds = np.interp(horizon, threat.times_s, threat.speeds_mps)
    correction = threat.center_x_m - threat.distance_used_m
    obstacles = xs - correction + speeds**2 / (2 * COMFORT_BRAKE_MPS2)
    obstacles += 50.0 * threat.release_fraction
    obstacles[(horizon < threat.entry_time_s) | (xs <= 0) | (obstacles < 0)] = np.inf
    return obstacles
