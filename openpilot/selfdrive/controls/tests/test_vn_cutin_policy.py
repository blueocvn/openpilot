import unittest
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.vn_cutin_policy import CutInPredictor


def model(*, side=1, approaching=True, probability=0.8, road_bend=0.0,
          x_std=2.0, y_std=0.3, second=None, overshoot=False, initial_lateral_m=3.0):
  path_x = np.arange(0.0, 61.0, 2.0)
  path_y = road_bend * path_x
  lead_x = [20.0, 28.0, 36.0, 44.0, 52.0, 60.0]
  offsets = [initial_lateral_m, -3.0 if overshoot else (0.5 if approaching else 4.0), -1.0 if approaching else 5.0,
             -2.0, -2.0, -2.0]
  hypothesis = SimpleNamespace(
    prob=probability, probTime=0.0, t=[0., 2., 4., 6., 8., 10.],
    x=lead_x, y=[road_bend * x + side * offset for x, offset in zip(lead_x, offsets, strict=True)],
    v=[4.] * 6, xStd=[x_std] * 6, yStd=[y_std] * 6,
  )
  return SimpleNamespace(position=SimpleNamespace(t=path_x / 8.33, x=path_x, y=path_y),
                         leadsV3=[hypothesis, *([second] if second is not None else [])])


def observed_lateral(lateral_m, *, probability=0.8, x_m=20.0, x_std=2.0, y_std=0.3):
  observed = model(approaching=False, probability=probability, initial_lateral_m=lateral_m,
                   x_std=x_std, y_std=y_std)
  lead = observed.leadsV3[0]
  lead.x = [x_m + 4.0 * t for t in lead.t]
  lead.y = [lateral_m] * len(lead.t)
  return observed


class TestCutInPredictor(unittest.TestCase):
  def confirmed(self, observed, *, predictor=None):
    predictor = predictor or CutInPredictor()
    for index in range(3):
      threat = predictor.update(observed, ego_speed_mps=8.33,
                                model_time_ns=1_000_000_000 + index * 50_000_000,
                                now_ns=1_000_000_000 + index * 50_000_000,
                                fresh=True, active=True)
    return predictor, threat

  def test_both_directions_confirm_after_three_fresh_frames(self):
    for side, label in ((1, "right_to_left"), (-1, "left_to_right")):
      predictor = CutInPredictor()
      observed = model(side=side)
      for index in range(2):
        threat = predictor.update(observed, ego_speed_mps=8.33,
                                  model_time_ns=1_000_000_000 + index * 50_000_000,
                                  now_ns=1_000_000_000 + index * 50_000_000,
                                  fresh=True, active=True)
        self.assertIsNone(threat)
      threat = predictor.update(observed, ego_speed_mps=8.33, model_time_ns=1_100_000_000,
                                now_ns=1_100_000_000, fresh=True, active=True)
      self.assertIsNotNone(threat)
      self.assertEqual(threat.side, label)
      self.assertEqual(threat.hypothesis_index, 0)
      self.assertGreater(threat.entry_time_s, 0.0)
      self.assertLessEqual(threat.entry_time_s, 2.0)

  def test_parallel_away_and_curved_road_do_not_create_false_cutin(self):
    for observed in (model(approaching=False), model(approaching=False, road_bend=0.10)):
      _, threat = self.confirmed(observed)
      self.assertIsNone(threat)

  def test_curved_road_uses_ego_path_not_zero_lateral(self):
    _, threat = self.confirmed(model(road_bend=0.10))
    self.assertIsNotNone(threat)

  def test_carla_ego_width_is_included_in_conflict_corridor(self):
    _, threat = self.confirmed(model(initial_lateral_m=1.7))
    self.assertIsNone(threat)

  def test_fast_crossing_between_sparse_two_second_samples_is_not_missed(self):
    _, threat = self.confirmed(model(overshoot=True))
    self.assertIsNotNone(threat)
    self.assertLess(threat.entry_time_s, 1.0)

  def test_hypothesis_slot_is_recorded_without_treating_slots_as_actor_ids(self):
    first = model(probability=0.3).leadsV3[0]
    second = model(side=-1).leadsV3[0]
    observed = model()
    observed.leadsV3 = [first, second]
    predictor, threat = self.confirmed(observed)
    self.assertEqual(threat.hypothesis_index, 1)
    self.assertEqual(predictor.diagnostic["hypothesis_index"], 1)

  def test_malformed_hypothesis_does_not_hide_a_valid_candidate(self):
    malformed = model(x_std=float("nan")).leadsV3[0]
    valid = model(side=-1).leadsV3[0]
    observed = model()
    observed.leadsV3 = [malformed, valid]
    predictor, threat = self.confirmed(observed)
    self.assertIsNotNone(threat)
    self.assertEqual(threat.side, "left_to_right")
    self.assertEqual(threat.hypothesis_index, 1)

  def test_consistent_frame_to_frame_lateral_motion_confirms_cutin(self):
    predictor = CutInPredictor()
    threat = None
    for index, lateral in enumerate((3.0, 2.7, 2.4)):
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed_lateral(lateral), ego_speed_mps=8.33,
                                model_time_ns=timestamp, now_ns=timestamp,
                                fresh=True, active=True)
    self.assertIsNotNone(threat)
    self.assertEqual(threat.side, "right_to_left")
    self.assertLess(threat.entry_time_s, 0.2)
    self.assertEqual(predictor.diagnostic["evidence"], "frame_motion")

  def test_confirmed_track_remains_an_obstacle_while_observed_inside_corridor(self):
    predictor = CutInPredictor()
    threat = None
    laterals = (3.0, 2.7, 2.4, 1.6, 1.3, 1.1, 0.9, 0.8, 0.7)
    for index, lateral in enumerate(laterals):
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed_lateral(lateral), ego_speed_mps=8.33,
                                model_time_ns=timestamp, now_ns=timestamp,
                                fresh=True, active=True)
    self.assertIsNotNone(threat)
    self.assertEqual(threat.entry_time_s, 0.0)
    self.assertTrue(np.isfinite(predictor.obstacle(np.array([0.0]))[0]))

  def test_confirmed_track_retracts_through_one_uncertain_frame_and_reacquires(self):
    predictor = CutInPredictor()
    for index, lateral in enumerate((3.0, 2.7, 2.4)):
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed_lateral(lateral), ego_speed_mps=8.33,
                                model_time_ns=timestamp, now_ns=timestamp,
                                fresh=True, active=True)
    self.assertIsNotNone(threat)
    threat = predictor.update(observed_lateral(2.1, x_std=20.0, y_std=2.0), ego_speed_mps=8.33,
                              model_time_ns=1_150_000_000, now_ns=1_150_000_000,
                              fresh=True, active=True)
    self.assertIsNotNone(threat)
    self.assertEqual(predictor.diagnostic["stage"], "retracting")
    threat = predictor.update(observed_lateral(1.8), ego_speed_mps=8.33,
                              model_time_ns=1_200_000_000, now_ns=1_200_000_000,
                              fresh=True, active=True)
    self.assertIsNotNone(threat)
    self.assertEqual(predictor.diagnostic["stage"], "confirmed")

  def test_frame_motion_away_from_corridor_does_not_confirm(self):
    predictor = CutInPredictor()
    for index, lateral in enumerate((2.4, 2.7, 3.0)):
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed_lateral(lateral), ego_speed_mps=8.33,
                                model_time_ns=timestamp, now_ns=timestamp,
                                fresh=True, active=True)
    self.assertIsNone(threat)

  def test_uncertain_early_observations_seed_track_but_only_reliable_frame_confirms(self):
    predictor = CutInPredictor()
    observations = ((4.0, 22.0, 20.0, 2.5), (3.7, 18.0, 20.0, 2.1), (3.4, 17.0, 2.0, 1.79))
    for index, (lateral, x_m, x_std, y_std) in enumerate(observations):
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed_lateral(lateral, x_m=x_m, x_std=x_std, y_std=y_std),
                                ego_speed_mps=8.33, model_time_ns=timestamp,
                                now_ns=timestamp, fresh=True, active=True)
      if index < 2:
        self.assertIsNone(threat)
    self.assertIsNotNone(threat)
    self.assertEqual(predictor.diagnostic["evidence"], "frame_motion")

  def test_frame_motion_with_broad_current_lateral_uncertainty_does_not_confirm(self):
    predictor = CutInPredictor()
    for index, lateral in enumerate((3.0, 2.7, 2.4)):
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed_lateral(lateral, y_std=1.9), ego_speed_mps=8.33,
                                model_time_ns=timestamp, now_ns=timestamp,
                                fresh=True, active=True)
    self.assertIsNone(threat)
    self.assertEqual(predictor.diagnostic["reason"], "motion_uncertainty")

  def test_nearby_temporal_hypotheses_do_not_break_frame_association(self):
    predictor = CutInPredictor()
    threat = None
    for index, lateral in enumerate((3.0, 2.7, 2.4)):
      observed = observed_lateral(lateral)
      alternatives = []
      for probability, offset, prob_time in ((0.75, 0.08, 2.0), (0.70, 0.16, 4.0)):
        alternative = observed_lateral(lateral + offset, probability=probability, x_m=20.1).leadsV3[0]
        alternative.probTime = prob_time
        alternatives.append(alternative)
      observed.leadsV3.extend(alternatives)
      timestamp = 1_000_000_000 + index * 50_000_000
      threat = predictor.update(observed, ego_speed_mps=8.33,
                                model_time_ns=timestamp, now_ns=timestamp,
                                fresh=True, active=True)
    self.assertIsNotNone(threat)
    self.assertEqual(threat.side, "right_to_left")
    self.assertEqual(threat.prob_time_s, 0.0)

  def test_repeated_frame_does_not_confirm_or_advance_time(self):
    predictor = CutInPredictor()
    observed = model()
    for index in range(4):
      threat = predictor.update(observed, ego_speed_mps=8.33, model_time_ns=1_000_000_000,
                                now_ns=1_000_000_000 + index * 50_000_000,
                                fresh=index == 0, active=True)
      self.assertIsNone(threat)

  def test_stale_nonfinite_and_ambiguous_data_reset_confirmation(self):
    for bad_model, age_ns in ((model(probability=float("nan")), 0),
                              (model(x_std=float("nan")), 0),
                              (model(y_std=2.0), 0),
                              (model(), 300_000_000)):
      predictor = CutInPredictor()
      predictor.update(model(), ego_speed_mps=8.33, model_time_ns=1_000_000_000,
                       now_ns=1_000_000_000, fresh=True, active=True)
      threat = predictor.update(bad_model, ego_speed_mps=8.33,
                                model_time_ns=1_050_000_000,
                                now_ns=1_050_000_000 + age_ns, fresh=True, active=True)
      self.assertIsNone(threat)
      self.assertEqual(predictor.diagnostic["stage"], "rejected")
    second = model(side=-1).leadsV3[0]
    _, threat = self.confirmed(model(second=second))
    self.assertIsNone(threat)

  def test_obstacle_appears_only_at_predicted_overlap(self):
    predictor, threat = self.confirmed(model())
    horizon = np.array([0., 0.5, 1., 1.5, 2.])
    obstacle = predictor.obstacle(horizon)
    self.assertIsNotNone(threat)
    self.assertTrue(np.all(np.isinf(obstacle[horizon < threat.entry_time_s])))
    self.assertTrue(np.all(np.isfinite(obstacle[horizon >= threat.entry_time_s])))
    self.assertAlmostEqual(threat.center_x_m, 18.48, places=2)
    self.assertAlmostEqual(threat.distance_used_m, 16.48, places=2)
    self.assertLess(obstacle[-1], 29.68)

  def test_stale_data_drops_obstacle_immediately(self):
    predictor, _ = self.confirmed(model())
    threat = predictor.update(model(), ego_speed_mps=8.33, model_time_ns=1_100_000_000,
                              now_ns=1_400_000_000, fresh=False, active=True)
    self.assertIsNone(threat)
    self.assertIsNone(predictor.obstacle(np.array([0., 1., 2.])))

  def test_missing_model_message_drops_confirmed_obstacle_immediately(self):
    predictor, _ = self.confirmed(model())
    threat = predictor.update(model(), ego_speed_mps=8.33, model_time_ns=1_100_000_000,
                              now_ns=1_150_000_000, fresh=False, active=True)
    self.assertIsNone(threat)
    self.assertIsNone(predictor.obstacle(np.array([0., 1., 2.])))

  def test_duplicate_timestamp_drops_confirmed_obstacle(self):
    predictor, _ = self.confirmed(model())
    threat = predictor.update(model(), ego_speed_mps=8.33, model_time_ns=1_100_000_000,
                              now_ns=1_150_000_000, fresh=True, active=True)
    self.assertIsNone(threat)
    self.assertEqual(predictor.diagnostic["reason"], "nonprogressing_model_time")

  def test_actor_behind_at_late_horizon_cannot_create_negative_mpc_obstacle(self):
    observed = model()
    lead = observed.leadsV3[0]
    lead.x = [20., 14., 8., 2., -4., -10.]
    predictor, threat = self.confirmed(observed)
    self.assertIsNotNone(threat)
    obstacle = predictor.obstacle(np.array([0., 1., 2., 4., 6., 8., 10.]))
    self.assertFalse(np.any(obstacle < 0))
    self.assertTrue(np.isinf(obstacle[-1]))


if __name__ == "__main__":
  unittest.main()
