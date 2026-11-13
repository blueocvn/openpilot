#!/usr/bin/env python3
"""Vision-only lead refinements: distance margin, speed from the distance trend, lead hold.

  PYTHONPATH=<bukapilot root> python3 -m unittest selfdrive.controls.tests.test_vision_lead -v
"""
import unittest
from types import SimpleNamespace

from selfdrive.config import RADAR_TO_CAMERA
from selfdrive.controls.lib.radar_helpers import (LEAD_HOLD_S, LEAD_MARGIN_MAX, LEAD_MARGIN_MIN, VisionLeadFilter)
from selfdrive.controls.radard import get_lead

DT = 0.05


def lead(x=30.0, v=0.0, prob=0.9, x_std=1.0, y=0.0):
  return SimpleNamespace(x=[x], y=[y], v=[v], xStd=[x_std], yStd=[0.5], vStd=[0.5], prob=prob)


class TestFlagsOff(unittest.TestCase):
  def test_no_filter_gives_the_model_lead_unchanged(self):
    got = get_lead(10.0, True, [], lead(x=30.0, v=7.0, x_std=12.0))
    self.assertAlmostEqual(got["dRel"], 30.0 - RADAR_TO_CAMERA)
    self.assertAlmostEqual(got["vLead"], 7.0)

  def test_filter_with_every_part_off_gives_the_same_lead(self):
    reference = get_lead(10.0, True, [], lead(x=30.0, v=7.0, x_std=12.0))
    got = get_lead(10.0, True, [], lead(x=30.0, v=7.0, x_std=12.0), vision_filter=VisionLeadFilter(DT), slot=0)
    for key in ("dRel", "yRel", "vRel", "vLead", "vLeadK", "modelProb", "status"):
      self.assertAlmostEqual(float(got[key]), float(reference[key]), msg=key)

  def test_low_probability_is_no_lead(self):
    self.assertFalse(get_lead(10.0, True, [], lead(prob=0.4), vision_filter=VisionLeadFilter(DT))["status"])
    self.assertFalse(get_lead(10.0, True, [], lead(prob=0.4))["status"])


class TestDistanceMargin(unittest.TestCase):
  def dist(self, x_std):
    f = VisionLeadFilter(DT, margin=True)
    return f.update(0, lead(x=30.0, x_std=x_std), 10.0)["dRel"]

  def test_margin_follows_xstd_between_the_limits(self):
    base = 30.0 - RADAR_TO_CAMERA
    self.assertAlmostEqual(self.dist(0.5), base - LEAD_MARGIN_MIN)
    self.assertAlmostEqual(self.dist(5.0), base - 5.0)
    self.assertAlmostEqual(self.dist(40.0), base - LEAD_MARGIN_MAX)

  def test_distance_never_negative(self):
    f = VisionLeadFilter(DT, margin=True)
    self.assertEqual(f.update(0, lead(x=3.0, x_std=1.0), 5.0)["dRel"], 0.0)

  def test_speed_is_untouched(self):
    f = VisionLeadFilter(DT, margin=True)
    self.assertAlmostEqual(f.update(0, lead(x=30.0, v=7.0), 10.0)["vLead"], 7.0)


class TestSpeedFromDistance(unittest.TestCase):
  def approach(self, v_ego, v_model, frames, x_std=2.0, v_lead_true=0.0, x0=40.0):
    f = VisionLeadFilter(DT, speed=True)
    out = None
    for i in range(frames):
      x = x0 + RADAR_TO_CAMERA - (v_ego - v_lead_true) * DT * i
      out = f.update(0, lead(x=x, v=v_model, x_std=x_std), v_ego)
    return out

  def test_stationary_object_reported_as_moving_is_seen_stationary(self):
    out = self.approach(v_ego=8.0, v_model=7.0, frames=30)
    self.assertLess(out["vLead"], 0.5)

  def test_never_above_the_model_speed(self):
    out = self.approach(v_ego=8.0, v_model=3.0, frames=30, v_lead_true=12.0)   # pulling away: trend says 12 m/s
    self.assertAlmostEqual(out["vLead"], 3.0)

  def test_no_effect_before_the_trend_has_warmed_up(self):
    out = self.approach(v_ego=8.0, v_model=7.0, frames=2)
    self.assertAlmostEqual(out["vLead"], 7.0)

  def test_uncertain_distance_resets_the_trend(self):
    out = self.approach(v_ego=8.0, v_model=7.0, frames=30, x_std=12.0)
    self.assertAlmostEqual(out["vLead"], 7.0)

  def test_relative_speed_follows_the_lowered_speed(self):
    out = self.approach(v_ego=8.0, v_model=7.0, frames=30)
    self.assertAlmostEqual(out["vRel"], out["vLead"] - 8.0)

  def test_steady_follow_of_a_moving_lead_keeps_its_speed(self):
    out = self.approach(v_ego=11.0, v_model=11.0, frames=40, v_lead_true=11.0)
    self.assertAlmostEqual(out["vLead"], 11.0, delta=0.5)


class TestLeadHold(unittest.TestCase):
  def test_lead_is_kept_for_the_hold_time_then_dropped(self):
    f = VisionLeadFilter(DT, hold=True)
    f.update(0, lead(x=30.0, v=0.0, prob=0.9), 10.0)
    frames = int(LEAD_HOLD_S / DT)
    for _ in range(frames):
      self.assertTrue(f.update(0, lead(prob=0.3), 10.0)["status"])
    self.assertFalse(f.update(0, lead(prob=0.3), 10.0)["status"])

  def test_held_lead_is_extrapolated_with_its_relative_speed(self):
    f = VisionLeadFilter(DT, hold=True)
    first = f.update(0, lead(x=30.0, v=0.0, prob=0.9), 10.0)
    held = f.update(0, lead(prob=0.3), 10.0)
    self.assertAlmostEqual(held["dRel"], first["dRel"] - 10.0 * DT)
    self.assertAlmostEqual(held["vLead"], 0.0)

  def test_released_at_once_when_the_probability_collapses(self):
    f = VisionLeadFilter(DT, hold=True)
    f.update(0, lead(prob=0.9), 10.0)
    self.assertFalse(f.update(0, lead(prob=0.05), 10.0)["status"])
    self.assertFalse(f.update(0, lead(prob=0.3), 10.0)["status"])   # nothing left to hold

  def test_slots_are_independent_and_a_new_lead_restarts_the_hold(self):
    f = VisionLeadFilter(DT, hold=True)
    f.update(0, lead(prob=0.9), 10.0)
    self.assertFalse(f.update(1, lead(prob=0.3), 10.0)["status"])
    for _ in range(5):
      f.update(0, lead(prob=0.3), 10.0)
    f.update(0, lead(prob=0.9), 10.0)
    self.assertTrue(f.update(0, lead(prob=0.3), 10.0)["status"])

  def test_hold_off_drops_the_lead_immediately(self):
    f = VisionLeadFilter(DT)
    f.update(0, lead(prob=0.9), 10.0)
    self.assertFalse(f.update(0, lead(prob=0.3), 10.0)["status"])


if __name__ == "__main__":
  unittest.main()
