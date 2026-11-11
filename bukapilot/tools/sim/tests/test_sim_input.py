#!/usr/bin/env python3
"""Offline tests for the CARLA-window keys, the blinker latch / steering nudge and the lane change gap.

  PYTHONPATH=<bukapilot root> python3 -m unittest discover -s tools/sim/tests -t . -v
"""
import unittest

from selfdrive.controls.lib.lane_change_gap import LaneChangeGap, phase_for, read_gap_tf
from tools.sim.bridge.common import (LONG_PRESS_FRAMES, SHORT_PRESS_FRAMES, SimulatorBridge, next_cruise_press, plan_manual_routing, toggle_blinker, update_blinker_latch)
from tools.sim.lib.x11_keys import DEFAULT_WINDOW_TITLE, REPEAT_PERIOD, X11Keys, key_events, pressed_keycodes


class TestX11Keys(unittest.TestCase):
  def test_pressed_keycodes(self):
    keymap = bytearray(32)
    keymap[0] = 0b00000100   # keycode 2
    keymap[3] = 0b10000001   # keycodes 24 and 31
    self.assertEqual(pressed_keycodes(bytes(keymap)), {2, 24, 31})

  def test_no_events_without_focus(self):
    self.assertEqual(key_events(set(), {"z", "w"}, False, {}, 10.0), ([], {}))

  def test_toggle_key_fires_once_per_press(self):
    chars, sent = key_events(set(), {"z"}, True, {}, 10.0)
    self.assertEqual(chars, ["z"])
    self.assertEqual(key_events({"z"}, {"z"}, True, sent, 10.5)[0], [])

  def test_key_already_down_when_focus_arrives_is_not_a_press(self):
    # prev_down comes from the raw key state, so z held before focusing is not new
    self.assertEqual(key_events({"z"}, {"z"}, True, {}, 10.0)[0], [])

  def test_drive_key_repeats_while_held(self):
    chars, sent = key_events({"a"}, {"a"}, True, {}, 10.0)
    self.assertEqual(chars, ["a"])
    self.assertEqual(key_events({"a"}, {"a"}, True, sent, 10.0 + REPEAT_PERIOD / 2)[0], [])
    self.assertEqual(key_events({"a"}, {"a"}, True, sent, 10.0 + REPEAT_PERIOD + 0.001)[0], ["a"])

  def test_released_drive_key_stops_repeating(self):
    _, sent = key_events(set(), {"a"}, True, {}, 10.0)
    chars, sent = key_events({"a"}, set(), True, sent, 11.0)
    self.assertEqual((chars, sent), ([], {}))


class TestWindowTitles(unittest.TestCase):
  def matches(self, spec, name):
    keys = X11Keys.__new__(X11Keys)  # no X display needed for the title rule
    keys.titles = [t.strip().lower() for t in spec.split(",") if t.strip()]
    return keys.title_matches(name)

  def test_default_watches_carla_and_the_exact_ui_window(self):
    self.assertTrue(self.matches(DEFAULT_WINDOW_TITLE, "CarlaUE4 "))
    self.assertTrue(self.matches(DEFAULT_WINDOW_TITLE, "UI"))
    self.assertFalse(self.matches(DEFAULT_WINDOW_TITLE, "Guide - Chromium"))
    self.assertFalse(self.matches(DEFAULT_WINDOW_TITLE, "bash"))


class TestCruiseRamp(unittest.TestCase):
  def test_long_press_while_five_or_more_below_target(self):
    self.assertEqual(next_cruise_press(40, 80), LONG_PRESS_FRAMES)
    self.assertEqual(next_cruise_press(75, 80), LONG_PRESS_FRAMES)

  def test_short_press_for_the_last_few_kph(self):
    self.assertEqual(next_cruise_press(77, 80), SHORT_PRESS_FRAMES)

  def test_nothing_once_the_target_is_reached(self):
    self.assertIsNone(next_cruise_press(80, 80))
    self.assertIsNone(next_cruise_press(90, 80))


class TestBlinkerLatch(unittest.TestCase):
  def test_toggle_and_side_switch(self):
    self.assertEqual(toggle_blinker(None, "left"), "left")
    self.assertEqual(toggle_blinker("left", "left"), None)
    self.assertEqual(toggle_blinker("left", "right"), "right")

  def test_not_cleared_before_lane_change_starts(self):
    self.assertEqual(update_blinker_latch("left", False, True), ("left", False))

  def test_cleared_after_lane_change_ends(self):
    latch, seen = update_blinker_latch("left", False, False)
    self.assertEqual((latch, seen), ("left", True))
    self.assertEqual(update_blinker_latch(latch, seen, True), (None, False))

  def test_off_latch_stays_off(self):
    self.assertEqual(update_blinker_latch(None, True, False), (None, False))


class TestManualRouting(unittest.TestCase):
  def test_nudge_in_blinker_direction_keeps_openpilot_steering_and_pedals(self):
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", True), (False, False))
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "right", True), (False, False))

  def test_nudge_with_pedal_key_keeps_pedals_manual(self):
    self.assertEqual(plan_manual_routing(True, -0.15, 1.0, 0.0, "left", True), (True, False))

  def test_wrong_direction_no_blinker_or_not_engaged_override_everything(self):
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "left", True), (True, True))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, None, True), (True, True))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", False), (True, True))

  def test_no_key_press_overrides_nothing(self):
    self.assertEqual(plan_manual_routing(False, 0.0, 0.0, 0.0, "left", True), (False, False))

  def test_left_key_gives_positive_torque_that_means_left(self):
    # a -> steer_-0.15, torque = steer * MANUAL_TORQUE; desire_helper reads torque > 0 as left
    from tools.sim.lib.keyboard_ctrl import KEY_COMMANDS
    steer = float(KEY_COMMANDS["a"].split("_")[1])
    torque = steer * SimulatorBridge.MANUAL_TORQUE
    self.assertGreater(torque, 1200)  # Honda Civic STEER_THRESHOLD default
    self.assertLess(float(KEY_COMMANDS["d"].split("_")[1]) * SimulatorBridge.MANUAL_TORQUE, -1200)


class TestLaneChangeGap(unittest.TestCase):
  DT = 0.05

  def test_unset_is_disabled_and_unchanged(self):
    gap = LaneChangeGap.__new__(LaneChangeGap)  # bypass the environment lookup
    gap.gap_tf, gap.tf = None, None
    self.assertFalse(gap.enabled)
    self.assertEqual(gap.update(1.6, True, True, True, self.DT), (1.6, False))

  def test_read_gap_tf(self):
    self.assertIsNone(read_gap_tf("x"))
    self.assertIsNone(read_gap_tf("-1"))
    self.assertEqual(read_gap_tf("3.0"), 3.0)

  def test_phases(self):
    self.assertEqual(phase_for(False, True, False), "idle")
    self.assertEqual(phase_for(True, False, False), "idle")
    self.assertEqual(phase_for(True, True, False), "open")
    self.assertEqual(phase_for(False, True, True), "cross")

  def test_headway_ramps_up_with_blinker_and_not_past_target(self):
    gap = LaneChangeGap(gap_tf=3.0)
    tf = 1.6
    for _ in range(int(10 / self.DT)):
      tf, ignore = gap.update(1.6, True, True, False, self.DT)
      self.assertFalse(ignore)
    self.assertAlmostEqual(tf, 3.0)
    gap = LaneChangeGap(gap_tf=3.0)
    tf, _ = gap.update(1.6, True, True, False, 1.0)
    self.assertAlmostEqual(tf, 1.9)  # 0.3 s of headway per second

  def test_crossing_holds_headway_and_ignores_lead(self):
    gap = LaneChangeGap(gap_tf=3.0)
    for _ in range(int(10 / self.DT)):
      gap.update(1.6, True, True, False, self.DT)
    tf, ignore = gap.update(1.6, True, True, True, self.DT)
    self.assertEqual((tf, ignore), (3.0, True))

  def test_ramps_back_to_normal_after_the_lane_change(self):
    gap = LaneChangeGap(gap_tf=3.0)
    for _ in range(int(10 / self.DT)):
      gap.update(1.6, True, True, False, self.DT)
    for _ in range(int(2.5 / self.DT)):
      tf, ignore = gap.update(1.6, False, True, False, self.DT)
    self.assertEqual((tf, ignore), (1.6, False))

  def test_gap_not_larger_than_normal_is_never_applied(self):
    gap = LaneChangeGap(gap_tf=1.0)
    tf, _ = gap.update(1.6, True, True, False, 1.0)
    self.assertEqual(tf, 1.6)


if __name__ == "__main__":
  unittest.main()
