from types import SimpleNamespace

import numpy as np

from openpilot.tools.sim.lib.carla_radar import RadarFrame
from openpilot.selfdrive.controls.lib.vn_radar_cutin_policy import RadarCutInPredictor


def model_path(curve=0.0):
  xs = np.linspace(0, 50, 51)
  return SimpleNamespace(position=SimpleNamespace(x=xs, y=curve * xs * xs))


def frame(number, time_ns, x, y, radial=-4.0, *, clutter=False):
  points = [(x - 0.2, y - 0.15, -0.15, radial),
            (x + 0.2, y + 0.15, 0.1, radial + 0.1)]
  if clutter:
    points += [(10 + i, (-1)**i * 4, -1.22, 0.0) for i in range(20)]
  return RadarFrame(number, time_ns, tuple(points))


def feed(predictor, measurements, *, speed=8.0, curve=0.0):
  for number, x, y in measurements:
    time_ns = 1_000_000_000 + number * 50_000_000
    predictor.update(frame(number, time_ns, x, y, clutter=True), model_path(curve),
                     ego_speed_mps=speed, now_ns=time_ns + 10_000_000, active=True)
  return predictor


def test_sustained_fresh_radar_frames_confirm_each_inward_direction():
  right = feed(RadarCutInPredictor(), [(0, 12, 3.7), (1, 11.8, 3.5),
                                       (2, 11.6, 3.3), (3, 11.4, 3.1), (4, 11.2, 2.9)])
  assert right.threat is not None
  assert right.threat.side == 'right_to_left'
  assert 0 < right.threat.entry_time_s < 1
  assert np.isinf(right.obstacle(np.array([0.0]))[0])
  assert np.isfinite(right.obstacle(np.array([1.0]))[0])

  left = feed(RadarCutInPredictor(), [(0, 12, -3.7), (1, 11.8, -3.5),
                                      (2, 11.6, -3.3), (3, 11.4, -3.1), (4, 11.2, -2.9)])
  assert left.threat is not None
  assert left.threat.side == 'left_to_right'


def test_parallel_outward_and_ground_only_do_not_trigger():
  for ys in ([3.5, 3.5, 3.5], [3.1, 3.3, 3.5]):
    predictor = feed(RadarCutInPredictor(), list(zip(range(3), [12, 11.8, 11.6], ys, strict=True)))
    assert predictor.threat is None
  predictor = RadarCutInPredictor()
  for number in range(3):
    time_ns = 1_000_000_000 + number * 50_000_000
    predictor.update(RadarFrame(number, time_ns, tuple((10+i, 3, -1.22, 0) for i in range(10))),
                     model_path(), ego_speed_mps=8, now_ns=time_ns, active=True)
  assert predictor.threat is None


def test_duplicate_and_expired_frames_cannot_sustain_threat():
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.5), (1, 11.8, 3.3)])
  duplicate = frame(1, 1_050_000_000, 11.8, 3.3)
  predictor.update(duplicate, model_path(), ego_speed_mps=8, now_ns=1_110_000_000, active=True)
  assert predictor.threat is None
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.5), (1, 11.8, 3.3), (2, 11.6, 3.1)])
  predictor.update(None, model_path(), ego_speed_mps=8, now_ns=1_400_000_000, active=True)
  assert predictor.threat is None


def test_curve_offset_does_not_make_parallel_object_look_inward():
  # y follows the camera-predicted path at each observed x, at a fixed offset.
  curve = 0.015
  xs = [12, 11.8, 11.6]
  predictor = feed(RadarCutInPredictor(), [(i, x, 3.5 + curve*x*x) for i, x in enumerate(xs)], curve=curve)
  assert predictor.threat is None


def test_far_lateral_and_far_forward_movers_are_not_imminent_conflicts():
  for initial_x, initial_y in ((33, -5.8), (28, -3.5)):
    predictor = feed(RadarCutInPredictor(), [(0, initial_x, initial_y),
                                             (1, initial_x - 0.2, initial_y + 0.2),
                                             (2, initial_x - 0.4, initial_y + 0.4)])
    assert predictor.threat is None


def test_single_lateral_step_does_not_count_as_sustained_cut_in():
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.5), (1, 11.8, 3.5), (2, 11.6, 3.3)])
  assert predictor.threat is None


def test_three_frames_are_not_enough_when_reflections_can_shift():
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.5), (1, 11.8, 3.3), (2, 11.6, 3.1)])
  assert predictor.threat is None


def test_path_shift_cannot_turn_raw_outward_motion_into_cut_in():
  predictor = RadarCutInPredictor()
  for number in range(5):
    time_ns = 1_000_000_000 + number * 50_000_000
    model = model_path()
    model.position.y = model.position.y - 0.2 * number
    predictor.update(frame(number, time_ns, 12 - 0.2 * number, -2.5 - 0.05 * number),
                     model, ego_speed_mps=8, now_ns=time_ns, active=True)
  assert predictor.threat is None


def test_inward_motion_with_distant_closing_time_does_not_require_braking():
  measurements = [(i, 14 - 0.1*i, 3.7 - 0.2*i) for i in range(5)]
  predictor = RadarCutInPredictor()
  for number, x, y in measurements:
    time_ns = 1_000_000_000 + number * 50_000_000
    predictor.update(frame(number, time_ns, x, y, radial=-1.0), model_path(),
                     ego_speed_mps=8, now_ns=time_ns, active=True)
  assert predictor.threat is None


def test_confirmed_track_survives_one_noisy_but_fresh_measurement():
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.7), (1, 11.8, 3.5),
                                           (2, 11.6, 3.3), (3, 11.4, 3.1), (4, 11.2, 2.9)])
  first_id = predictor.threat.track_id
  time_ns = 1_250_000_000
  predictor.update(frame(5, time_ns, 11.0, 2.92), model_path(),
                   ego_speed_mps=8, now_ns=time_ns, active=True)
  assert predictor.threat is not None
  assert predictor.threat.track_id == first_id


def test_confirmed_track_survives_sub_meter_reflection_shift():
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.7), (1, 11.8, 3.5),
                                           (2, 11.6, 3.3), (3, 11.4, 3.1), (4, 11.2, 2.9)])
  first_id = predictor.threat.track_id
  time_ns = 1_250_000_000
  predictor.update(frame(5, time_ns, 11.0, 2.43), model_path(),
                   ego_speed_mps=8, now_ns=time_ns, active=True)
  assert predictor.threat is not None
  assert predictor.threat.track_id == first_id


def test_two_plausible_clusters_cannot_reuse_one_confirmed_identity():
  predictor = feed(RadarCutInPredictor(), [(0, 12, 3.7), (1, 11.8, 3.5),
                                           (2, 11.6, 3.3), (3, 11.4, 3.1), (4, 11.2, 2.9)])
  time_ns = 1_250_000_000
  points = [(10.8, 2.55, 0, -4), (11.2, 2.85, 0.1, -4),
            (12.2, 3.45, 0, -4), (12.6, 3.75, 0.1, -4)]
  predictor.update(RadarFrame(5, time_ns, tuple(points)), model_path(),
                   ego_speed_mps=8, now_ns=time_ns, active=True)
  assert predictor.threat is None


def test_noisy_reflections_can_confirm_sustained_inward_trend():
  ys = [4.3, 4.25, 4.3, 4.1, 4.05, 3.8, 3.9, 3.92, 3.5]
  predictor = feed(RadarCutInPredictor(), [(i, 12 - 0.2*i, y) for i, y in enumerate(ys)])
  assert predictor.threat is not None
  assert predictor.threat.side == "right_to_left"


def test_lateral_reflection_oscillation_alone_does_not_confirm():
  ys = [3.5, 3.4, 3.6, 3.45, 3.55, 3.4, 3.6, 3.45, 3.5]
  predictor = feed(RadarCutInPredictor(), [(i, 12 - 0.2*i, y) for i, y in enumerate(ys)])
  assert predictor.threat is None
