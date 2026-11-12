#!/usr/bin/env python3
"""Offline tests for the CARLA-window keys, the blinker latch / steering nudge and the lane change speed gate.

  PYTHONPATH=<bukapilot root> python3 -m unittest discover -s tools/sim/tests -t . -v
"""
import unittest

from selfdrive.controls.lib.desire_helper import lane_change_speed_min
from tools.sim.bridge.common import (LONG_PRESS_FRAMES, SHORT_PRESS_FRAMES, SimulatorBridge, blinker_for_command, lead_truth, collision_banner, frame_label, frame_name, save_lead_frame, next_cruise_press, plan_manual_routing, status_line)
from tools.sim.lib.keyboard_ctrl import KEY_COMMANDS
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
  def test_side_keys_latch_and_center_key_turns_off(self):
    self.assertEqual(blinker_for_command("left"), "left")
    self.assertEqual(blinker_for_command("right"), "right")
    self.assertIsNone(blinker_for_command("off"))

  def test_keys_map_to_blinker_commands(self):
    self.assertEqual(KEY_COMMANDS["z"], "blinker_left")
    self.assertEqual(KEY_COMMANDS["x"], "blinker_off")
    self.assertEqual(KEY_COMMANDS["c"], "blinker_right")


class TestLeadTruth(unittest.TestCase):
  EGO = dict(ego_xy=(0.0, 0.0), ego_forward_xy=(1.0, 0.0), ego_half_length=2.0)

  def test_nearest_in_lane_bumper_gap_and_speed(self):
    others = [((30.0, 0.5), (8.0, 0.0), 1.0), ((12.0, -1.0), (5.0, 0.0), 1.0)]
    gap, v_lead = lead_truth(others=others, **self.EGO)
    self.assertAlmostEqual(gap, 12.0 - 2.0 - 1.0)
    self.assertAlmostEqual(v_lead, 5.0)

  def test_ignores_other_lane_and_behind(self):
    others = [((20.0, 3.5), (5.0, 0.0), 1.0), ((-10.0, 0.0), (5.0, 0.0), 1.0)]
    self.assertIsNone(lead_truth(others=others, **self.EGO))

  def test_speed_is_along_ego_heading(self):
    gap, v_lead = lead_truth((0.0, 0.0), (0.0, 1.0), 2.0, [((0.0, 20.0), (3.0, 4.0), 1.0)])
    self.assertAlmostEqual(v_lead, 4.0)


class TestStatusLine(unittest.TestCase):
  OP = dict(engaged=True, v_cruise=80.0, alert='', accel=0.8, steer=-1.3, throttle=0.67, brake=0.0,
            source='cruise', fcw=False)

  def test_sources_are_labelled_and_error_is_camera_minus_truth(self):
    line = status_line(self.OP, (118.0, 14.8, 0.59), 11.2, (94.7, 10.6))
    for tag in ("[OP cmd]", "[OP sees]", "[SIM]", "[ERR] dRel=+23.3m vLead=+4.2"):
      self.assertIn(tag, line)
    self.assertIn("closing=+0.6 ttc=", line)

  def test_missed_lead_and_contact(self):
    line = status_line(self.OP, None, 20.0, (-0.1, 10.0))
    self.assertIn("[OP sees] NO LEAD", line)
    self.assertIn("[ERR] lead missed", line)
    self.assertIn("ttc=CONTACT", line)

  def test_without_ground_truth_only_openpilot_side(self):
    line = status_line(self.OP, None, 11.2, None)
    self.assertNotIn("[ERR]", line)
    self.assertIn("[SIM] speed=11.2", line)


class TestLeadFrame(unittest.TestCase):
  def test_label_and_name_carry_truth_and_perception(self):
    self.assertEqual(frame_label((8.1, 10.7), (23.2, 17.6, 0.98)), "SIM gap=8.1m vLead=10.7  |  OP dRel=23.2m vLead=17.6 prob=0.98")
    self.assertIn("OP NO LEAD", frame_label((8.1, 10.7), None))
    self.assertEqual(frame_name(3, (8.1, 10.7), (23.2, 17.6, 0.98)), "0003_gap008.1_op23.2.jpg")
    self.assertEqual(frame_name(4, (-0.2, 10.7), None), "0004_gap000.0_opnone.jpg")

  def test_saved_frame_is_a_jpeg_of_the_same_size(self):
    import pathlib
    import tempfile

    import numpy as np
    from PIL import Image
    bgra = np.zeros((1208, 1928, 4), dtype=np.uint8)
    with tempfile.TemporaryDirectory() as tmp:
      path = pathlib.Path(tmp) / "sub" / "f.jpg"
      save_lead_frame(bgra, path, "SIM gap=1.0m")
      self.assertEqual(Image.open(path).size, (1928, 1208))


class TestCollisionBanner(unittest.TestCase):
  def test_banner_is_upper_case_and_says_what_openpilot_saw(self):
    banner = collision_banner("vehicle.kawasaki.ninja", 1234.4, 20.4, (23.2, 17.6, 0.98))
    self.assertEqual(banner, banner.upper())
    for part in ("COLLISION", "COLLIDED WITH: VEHICLE.KAWASAKI.NINJA", "20.4 M/S", "1234", "DREL=23.2M"):
      self.assertIn(part, banner)

  def test_banner_when_openpilot_saw_nothing(self):
    self.assertIn("SAW NO LEAD", collision_banner("vehicle.kawasaki.ninja", 10.0, 5.0, None))


class TestManualRouting(unittest.TestCase):
  def test_nudge_in_blinker_direction_keeps_openpilot_steering_and_pedals(self):
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", True, True), (False, False))
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "right", True, True), (False, False))

  def test_nudge_with_pedal_key_keeps_pedals_manual(self):
    self.assertEqual(plan_manual_routing(True, -0.15, 1.0, 0.0, "left", True, True), (True, False))

  def test_wrong_direction_no_blinker_or_not_engaged_override_everything(self):
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "left", True, True), (True, True))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, None, True, True), (True, True))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", False, True), (True, True))

  def test_blinker_direction_steers_by_hand_when_no_lane_change_can_start(self):
    # lane change state off (for example below the lane change speed): the key must steer the car
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", True, False), (True, True))
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "right", True, False), (True, True))

  def test_no_key_press_overrides_nothing(self):
    self.assertEqual(plan_manual_routing(False, 0.0, 0.0, 0.0, "left", True, True), (False, False))

  def test_left_key_gives_positive_torque_that_means_left(self):
    # a -> steer_-0.15, torque = steer * MANUAL_TORQUE; desire_helper reads torque > 0 as left
    from tools.sim.lib.keyboard_ctrl import KEY_COMMANDS
    steer = float(KEY_COMMANDS["a"].split("_")[1])
    torque = steer * SimulatorBridge.MANUAL_TORQUE
    self.assertGreater(torque, 1200)  # Honda Civic STEER_THRESHOLD default
    self.assertLess(float(KEY_COMMANDS["d"].split("_")[1]) * SimulatorBridge.MANUAL_TORQUE, -1200)


class TestLaneChangeSpeedGate(unittest.TestCase):
  MPH_30 = 30 * 0.44704

  def test_simulator_override_is_in_kph(self):
    self.assertAlmostEqual(lane_change_speed_min("30"), 30 / 3.6)

  def test_unset_or_invalid_keeps_30_mph(self):
    for value in ("", "x", "0", "-5"):
      self.assertAlmostEqual(lane_change_speed_min(value), self.MPH_30, places=4)


if __name__ == "__main__":
  unittest.main()
