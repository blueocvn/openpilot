#!/usr/bin/env python3
"""Offline tests for the CARLA-window keys, the blinker latch / steering nudge and the lane change speed gate.

  PYTHONPATH=<bukapilot root> python3 -m unittest discover -s tools/sim/tests -t . -v
"""
import unittest

from selfdrive.controls.lib.desire_helper import lane_change_speed_min
from tools.sim.bridge.common import (LONG_PRESS_FRAMES, SHORT_PRESS_FRAMES, SimulatorBridge, blinker_for_command, lead_truth, nearest_ahead, collision_banner, frame_label, frame_name, save_lead_frame, next_cruise_press, next_cruise_action, plan_manual_routing, status_line, cruise_button_step, CRUISE_RELEASE_FRAMES)
from tools.sim.bridge.bench_summary import judge, rule_for, split_label, table_rows
from tools.sim.bridge.run_metrics import (OvertakeScript, RunMetrics, parse_obstacle_spec, project_half_length)
from tools.sim.lib.keyboard_ctrl import KEY_COMMANDS
from tools.sim.lib.x11_keys import (DEFAULT_WINDOW_TITLE, REPEAT_PERIOD, US_QWERTY_KEYCODES, X11Keys, build_keycode_map, char_to_keysym,
                                    key_events, pressed_keycodes)


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


class TestKeycodeMap(unittest.TestCase):
  def test_keysym_of_ascii_latin1_and_unicode_chars(self):
    self.assertEqual(char_to_keysym("w"), 0x77)
    self.assertEqual(char_to_keysym("é"), 0xe9)
    self.assertEqual(char_to_keysym("ư"), 0x01000000 + 0x1b0)

  def test_us_qwerty_positions_match_the_x11_keycodes(self):
    self.assertEqual([US_QWERTY_KEYCODES[c] for c in "wasd"], [25, 38, 39, 40])
    self.assertEqual([US_QWERTY_KEYCODES[c] for c in "123"], [10, 11, 12])

  def test_finds_the_plain_key_in_the_layout(self):
    # keycodes 38..40 carry a, s, d in group 1 (2 columns: plain, shift)
    keysyms = [0x61, 0x41, 0x73, 0x53, 0x64, 0x44]
    self.assertEqual(build_keycode_map("asd", keysyms, 38, 2), {38: "a", 39: "s", 40: "d"})

  def test_layout_without_ascii_w_falls_back_to_the_qwerty_position(self):
    # vietnamese style layout: the w key types ư and nothing carries an ascii w
    keysyms = [0x01000000 + 0x1b0, 0x01000000 + 0x1af, 0x61, 0x41]   # keycode 25 = ư, keycode 26 = a
    self.assertEqual(build_keycode_map("wa", keysyms, 25, 2), {26: "a", 25: "w"})

  def test_fallback_never_steals_a_key_used_by_another_char(self):
    # the layout puts ascii "a" on keycode 25 (the QWERTY w position) and has no w
    self.assertEqual(build_keycode_map("wa", [0x61, 0x41], 25, 2), {25: "a"})

  def test_unicode_char_is_found_through_its_unicode_keysym(self):
    keysyms = [0x01000000 + 0x1b0, 0x01000000 + 0x1af]
    self.assertEqual(build_keycode_map("ư", keysyms, 100, 2), {100: "ư"})

  def test_plain_column_wins_over_altgr(self):
    # "1" is at the AltGr column of keycode 10 and in the plain column of keycode 11: the plain key wins
    keysyms = [0x01000000 + 0x103, 0x01000000 + 0x102, 0x31, 0x21, 0x31, 0x21, 0x32, 0x40]
    self.assertEqual(build_keycode_map("1", keysyms, 10, 4), {11: "1"})


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


class TestCruiseRampDown(unittest.TestCase):
  def test_lowers_with_set_minus(self):
    from selfdrive.car.honda.values import CruiseButtons
    self.assertEqual(next_cruise_action(40, 30), (CruiseButtons.DECEL_SET, LONG_PRESS_FRAMES))
    self.assertEqual(next_cruise_action(33, 30), (CruiseButtons.DECEL_SET, SHORT_PRESS_FRAMES))
    self.assertEqual(next_cruise_action(40, 60), (CruiseButtons.RES_ACCEL, LONG_PRESS_FRAMES))
    self.assertIsNone(next_cruise_action(30, 30))
    self.assertIsNone(next_cruise_action(30.4, 30))


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

  def test_lateral_offset_is_positive_to_the_left_and_limited(self):
    # CARLA: x forward, y right, so an object at y = -2 is 2 m to the left
    ego = dict(ego_xy=(0.0, 0.0), ego_forward_xy=(1.0, 0.0), ego_half_length=2.0)
    self.assertAlmostEqual(nearest_ahead(others=[((20.0, -2.0), (0.0, 0.0), 1.0)], lateral_limit=6.0, **ego)[2], 2.0)
    self.assertAlmostEqual(nearest_ahead(others=[((20.0, 1.5), (0.0, 0.0), 1.0)], lateral_limit=6.0, **ego)[2], -1.5)
    self.assertIsNone(nearest_ahead(others=[((20.0, 7.0), (0.0, 0.0), 1.0)], lateral_limit=6.0, **ego))

  def test_box_half_length_along_the_ego_heading(self):
    self.assertAlmostEqual(project_half_length(2.2, 0.9, 0.0), 2.2)
    self.assertAlmostEqual(project_half_length(2.2, 0.9, 90.0), 0.9)
    self.assertAlmostEqual(project_half_length(2.2, 0.9, -90.0), 0.9)


class TestObstacleSpec(unittest.TestCase):
  def test_defaults_to_world_fixed(self):
    self.assertEqual(parse_obstacle_spec("container:200"), [("container", 200.0, 0.0)])

  def test_motorcycle_can_drive_and_others_stand_still(self):
    self.assertEqual(parse_obstacle_spec("motorcycle:150:7.2, barrier:300"),
                     [("motorcycle", 150.0, 7.2), ("barrier", 300.0, 0.0)])

  def test_rejects_unknown_kind_bad_distance_and_moving_non_motorcycle(self):
    for bad in ("tree:50", "car:0", "car:-5", "car:50:10", "car", "car:1:2:3"):
      with self.assertRaises(ValueError, msg=bad):
        parse_obstacle_spec(bad)

  def test_bridge_actors_pass_obstacles_apart_from_the_motorcycle_options(self):
    import argparse
    from tools.sim.run_bridge import bridge_actors
    args = argparse.Namespace(obstacle_spec="motorcycle:150:7.2,container:200", motorcycle=False, motorcycle_spec=None,
                              motorcycle_model=None)
    self.assertEqual(bridge_actors(args), (None, [("motorcycle", 150.0, 7.2), ("container", 200.0, 0.0)]))
    none = argparse.Namespace(obstacle_spec=None, motorcycle=False, motorcycle_spec=None, motorcycle_model=None)
    self.assertEqual(bridge_actors(none), (None, None))
    both = argparse.Namespace(obstacle_spec="barrier:100", motorcycle=False, motorcycle_spec="same:40:25",
                              motorcycle_model=None)
    self.assertEqual(bridge_actors(both), ({"specs": [("same", 40.0, 25.0)], "models": None}, [("barrier", 100.0, 0.0)]))


class TestRunMetrics(unittest.TestCase):
  def run_samples(self, metrics, speeds, gaps=None, set_kph=60.0, start=100.0):
    for i, speed in enumerate(speeds):
      truth = None if gaps is None or gaps[i] is None else (gaps[i], 0.0)
      metrics.sample(start + i * 0.1, speed, truth, (gaps[i], 0.0, 0.9) if truth else None, set_kph)

  def test_clock_starts_when_engaged(self):
    m = RunMetrics()
    m.sample(1.0, 5.0, None, None, 60.0, engaged=False)
    self.assertFalse(m.summary()["engaged"])
    m.sample(2.0, 5.0, None, None, 60.0)
    m.sample(3.0, 5.0, None, None, 60.0)
    self.assertAlmostEqual(m.summary()["duration_s"], 1.0)
    self.assertAlmostEqual(m.summary()["distance_m"], 5.0)

  def test_min_gap_first_lead_and_final_speed(self):
    m = RunMetrics()
    self.run_samples(m, [10.0, 10.0, 8.0, 4.0, 0.0], gaps=[None, 40.0, 25.0, 9.0, 4.4])
    summary = m.summary()
    self.assertEqual(summary["min_gap_m"], 4.4)
    self.assertEqual(summary["first_lead_true_gap_m"], 40.0)
    self.assertEqual(summary["final_speed_mps"], 0.0)

  def test_peak_decel_and_time_below_threshold(self):
    m = RunMetrics()
    speeds = [10.0 - 3.0 * 0.1 * i for i in range(20)]   # constant -3 m/s2 for 2 s
    self.run_samples(m, speeds, gaps=[30.0] * 20)
    summary = m.summary()
    self.assertAlmostEqual(summary["peak_decel_mps2"], -3.0, places=1)
    self.assertGreaterEqual(summary["time_below_m1_5_s"], 1.4)
    self.assertEqual(summary["false_brake_events"], 0)

  def test_false_braking_counts_events_on_a_clear_road_only(self):
    m = RunMetrics()
    speeds = [10.0 - 3.0 * 0.1 * i for i in range(10)] + [7.0] * 10 + [7.0 - 3.0 * 0.1 * i for i in range(10)]
    self.run_samples(m, speeds)   # nothing ahead the whole time
    self.assertEqual(m.summary()["false_brake_events"], 2)
    near = RunMetrics()
    self.run_samples(near, speeds, gaps=[30.0] * 30)   # an object 30 m ahead explains the braking
    self.assertEqual(near.summary()["false_brake_events"], 0)

  def test_collision_with_an_obstacle_ends_the_run_other_contacts_do_not(self):
    m = RunMetrics()
    self.run_samples(m, [10.0] * 5)
    m.collide("static.road.curb", 10.0)
    self.assertFalse(m.summary()["collision"])
    self.assertEqual(m.summary()["other_contacts"], 1)
    m.collide("vehicle.kawasaki.ninja", 9.0)
    self.assertTrue(m.summary()["collision"])
    self.assertEqual(m.summary()["impact_speed_mps"], 9.0)
    self.assertIsNone(m.done_reason())
    self.run_samples(m, [0.0] * 30, start=101.0)
    self.assertEqual(m.done_reason(), "collision")

  def test_impact_deceleration_is_not_scored_as_braking(self):
    m = RunMetrics()
    self.run_samples(m, [8.0] * 10, gaps=[5.0] * 10)
    m.collide("vehicle.kawasaki.ninja", 8.0)
    self.run_samples(m, [0.0] * 10, start=101.0)   # the bike is knocked away: nothing ahead, speed gone
    summary = m.summary()
    self.assertEqual(summary["peak_decel_mps2"], 0.0)
    self.assertEqual(summary["false_brake_events"], 0)

  def test_run_ends_when_stopped_for_good_or_distance_reached(self):
    stopped = RunMetrics()
    self.run_samples(stopped, [8.0] * 5 + [0.0] * 40)   # stopped for 4 s only
    self.assertIsNone(stopped.done_reason())
    self.run_samples(stopped, [0.0] * 40, start=110.0)   # now 8 s
    self.assertEqual(stopped.done_reason(), "stopped")
    far = RunMetrics(max_distance_m=50.0)
    self.run_samples(far, [10.0] * 60)
    self.assertEqual(far.done_reason(), "distance")

  def test_standing_at_the_start_is_not_a_stop(self):
    m = RunMetrics()
    self.run_samples(m, [0.0] * 100)
    self.assertIsNone(m.done_reason())

  def test_cruise_average_starts_once_90_percent_of_set_speed_is_reached(self):
    m = RunMetrics()
    self.run_samples(m, [0.0, 5.0, 10.0, 15.0, 15.0, 15.0, 15.0], set_kph=60.0)   # 60 km/h = 16.7 m/s, 90% = 15
    self.assertAlmostEqual(m.summary()["avg_speed_cruise_mps"], 15.0)
    never = RunMetrics()
    self.run_samples(never, [5.0] * 10, set_kph=60.0)
    self.assertIsNone(never.summary()["avg_speed_cruise_mps"])

  def test_overtake_brake_and_lateral_error(self):
    m = RunMetrics(stop_when_stopped=False)
    m.sample(1.0, 1.0, None, None, 30.0, brake=0.5, in_overtake=False)
    m.sample(1.1, 1.0, None, None, 30.0, brake=0.2, in_overtake=True, y_rel_pair=(1.5, 2.0))
    m.sample(1.2, 1.0, None, None, 30.0, brake=0.0, in_overtake=True, y_rel_pair=(3.0, 2.0))
    summary = m.summary()
    self.assertEqual(summary["overtake_max_brake"], 0.2)
    self.assertEqual(summary["overtake_yrel_samples"], 2)
    self.assertAlmostEqual(summary["overtake_yrel_mean_abs_error_m"], 0.75)


class TestOvertakeScript(unittest.TestCase):
  SEGMENTS = ((1.0, ("steer_-0.15", "throttle_1.0")), (1.0, ("steer_0.15",)))

  def test_waits_for_the_car_to_stand_still_then_plays_the_segments(self):
    script = OvertakeScript(self.SEGMENTS, still_s=2.0)
    self.assertEqual(script.commands(0.0, 5.0), [])
    self.assertEqual(script.commands(1.0, 0.0), [])
    self.assertEqual(script.commands(2.5, 0.0), [])
    self.assertEqual(script.commands(3.0, 0.0), ["steer_-0.15", "throttle_1.0"])
    self.assertTrue(script.active)

  def test_not_armed_until_the_car_has_driven(self):
    script = OvertakeScript(self.SEGMENTS, still_s=1.0)
    self.assertEqual(script.commands(0.0, 0.0, armed=False), [])
    self.assertEqual(script.commands(5.0, 0.0, armed=False), [])
    self.assertEqual(script.commands(5.1, 0.0, armed=True), [])
    self.assertEqual(script.commands(6.2, 0.0, armed=True), ["steer_-0.15", "throttle_1.0"])

  def test_commands_repeat_like_a_held_key_and_change_per_segment(self):
    script = OvertakeScript(self.SEGMENTS, still_s=0.0, repeat_s=0.1)
    self.assertEqual(script.commands(0.0, 0.0), ["steer_-0.15", "throttle_1.0"])
    self.assertEqual(script.commands(0.05, 0.0), [])
    self.assertEqual(script.commands(0.11, 0.0), ["steer_-0.15", "throttle_1.0"])
    self.assertEqual(script.commands(1.05, 0.0), ["steer_0.15"])
    self.assertEqual(script.commands(2.1, 0.0), [])
    self.assertFalse(script.active)
    self.assertTrue(script.finished_playing(2.1))

  def test_default_commands_are_real_keyboard_commands(self):
    keys = set(KEY_COMMANDS.values())
    for _, cmds in OvertakeScript.DEFAULT_SEGMENTS:
      for cmd in cmds:
        self.assertIn(cmd, keys)


class TestBenchSummary(unittest.TestCase):
  BASE = dict(engaged=True, collision=False, min_gap_m=6.0, peak_decel_mps2=-2.0, false_brake_events=0)

  def run_of(self, label, **fields):
    return {**self.BASE, "scenario": label, **fields}

  def test_label_split(self):
    self.assertEqual(split_label("motorcycle/20/30"), ("motorcycle", 20.0, 30.0, False))
    self.assertEqual(split_label("barrier/0/30/overtake"), ("barrier", 0.0, 30.0, True))
    self.assertEqual(split_label("none/0/60"), ("none", 0.0, 60.0, False))

  def test_the_cell_decides_the_rule(self):
    self.assertEqual(rule_for("none/0/60"), "clear")
    self.assertEqual(rule_for("container/0/30"), "stop")
    self.assertEqual(rule_for("motorcycle/0/60"), "stop")
    self.assertEqual(rule_for("motorcycle/7.2/60"), "stop")      # 2 m/s: stop behind it
    self.assertEqual(rule_for("motorcycle/20/30"), "follow")     # 5.6 m/s: follow it
    self.assertEqual(rule_for("motorcycle/40/60"), "follow")
    self.assertEqual(rule_for("barrier/0/30/overtake"), "overtake")
    self.assertEqual(rule_for("tree/0/30"), "unknown")

  def test_a_run_on_an_overloaded_machine_is_discarded(self):
    ok, reason = judge(self.run_of("barrier/0/60", realtime_factor=0.8))
    self.assertFalse(ok)
    self.assertIn("overloaded", reason)
    self.assertTrue(judge(self.run_of("barrier/0/60", realtime_factor=1.02))[0])

  def test_obstacle_rows_need_no_collision_and_a_4_m_gap(self):
    self.assertTrue(judge(self.run_of("barrier/0/60"))[0])
    self.assertFalse(judge(self.run_of("barrier/0/60", min_gap_m=3.9))[0])
    self.assertFalse(judge(self.run_of("container/0/60", collision=True, min_gap_m=-0.2, impact_speed_mps=9.0))[0])
    self.assertFalse(judge(self.run_of("car/0/60", peak_decel_mps2=-4.0))[0])
    self.assertFalse(judge(self.run_of("car/0/60", engaged=False))[0])
    self.assertFalse(judge(self.run_of("car/0/60", min_gap_m=None))[0])

  def test_leaving_the_road_is_a_failure_even_without_a_collision(self):
    ok, reason = judge(self.run_of("barrier/0/30", other_contacts=3))
    self.assertFalse(ok)
    self.assertIn("left the road", reason)

  def test_following_gap_within_20_percent_of_intended(self):
    self.assertTrue(judge(self.run_of("motorcycle/40/60", steady_gap_m=22.0, intended_gap_m=20.0))[0])
    self.assertFalse(judge(self.run_of("motorcycle/40/60", steady_gap_m=25.0, intended_gap_m=20.0))[0])
    self.assertFalse(judge(self.run_of("motorcycle/40/60"))[0])
    self.assertTrue(judge(self.run_of("motorcycle/20/30", steady_gap_m=15.0, intended_gap_m=14.0))[0])

  def test_clear_road_needs_no_false_braking_and_90_percent_speed(self):
    ok = self.run_of("none/0/30", avg_speed_cruise_mps=7.8)   # 90% of 30 km/h = 7.5 m/s
    self.assertTrue(judge(ok)[0])
    self.assertFalse(judge({**ok, "avg_speed_cruise_mps": 7.0})[0])
    self.assertFalse(judge({**ok, "false_brake_events": 1})[0])

  def test_overtake_needs_no_contact_and_no_braking_against_the_driver(self):
    self.assertTrue(judge(self.run_of("barrier/0/30/overtake", overtake_max_brake=0.0))[0])
    self.assertFalse(judge(self.run_of("barrier/0/30/overtake", overtake_max_brake=0.4))[0])
    self.assertFalse(judge(self.run_of("barrier/0/30/overtake", collision=True))[0])

  def test_table_groups_repeats_and_reports_the_spread(self):
    runs = []
    for gap in (5.0, 6.0, 8.0):
      run = self.run_of("barrier/0/30", min_gap_m=gap)
      run["passed"], run["reason"] = judge(run)
      runs.append(run)
    (row,) = table_rows(runs)
    self.assertEqual((row["obstacle"], row["obstacle_kph"], row["ego_kph"]), ("barrier", 0.0, 30.0))
    self.assertEqual((row["runs"], row["passed"], row["collisions"]), (3, 3, 0))
    self.assertEqual((row["min_gap_worst_m"], row["min_gap_median_m"], row["min_gap_spread_m"]), (5.0, 6.0, 3.0))


class TestSimParams(unittest.TestCase):
  def test_parses_boolean_flags(self):
    from tools.sim.lib.params import parse_sim_params
    self.assertEqual(parse_sim_params("VisionLeadHold=1, VisionLeadDistanceMargin=0"),
                     [("VisionLeadHold", True), ("VisionLeadDistanceMargin", False)])
    self.assertEqual(parse_sim_params(""), [])
    self.assertEqual(parse_sim_params(None), [])

  def test_rejects_anything_but_zero_or_one(self):
    from tools.sim.lib.params import parse_sim_params
    for bad in ("Key", "Key=2", "Key=true"):
      with self.assertRaises(ValueError, msg=bad):
        parse_sim_params(bad)


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

  def test_wrong_direction_no_blinker_or_not_engaged_steer_by_hand_but_keep_pedals(self):
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "left", True, True), (False, True))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, None, True, True), (False, True))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", False, True), (False, True))

  def test_blinker_direction_steers_by_hand_when_no_lane_change_can_start(self):
    # lane change state off (for example below the lane change speed): the key must steer the car,
    # openpilot keeps the pedals so it still holds the distance to the lead
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 0.0, "left", True, False), (False, True))
    self.assertEqual(plan_manual_routing(True, 0.15, 0.0, 0.0, "right", True, False), (False, True))

  def test_pedal_key_takes_the_pedals_even_without_a_steer_key(self):
    # a throttle key alone must not straighten the wheel
    self.assertEqual(plan_manual_routing(True, 0.0, 1.0, 0.0, None, True, False), (True, False))
    self.assertEqual(plan_manual_routing(True, 0.0, 0.0, 1.0, None, True, True), (True, False))
    self.assertEqual(plan_manual_routing(True, -0.15, 0.0, 1.0, "left", True, False), (True, True))

  def test_no_key_press_overrides_nothing(self):
    self.assertEqual(plan_manual_routing(False, 0.0, 0.0, 0.0, "left", True, True), (False, False))

  def test_left_key_gives_positive_torque_that_means_left(self):
    # a -> steer_-0.15, torque = steer * MANUAL_TORQUE; desire_helper reads torque > 0 as left
    from tools.sim.lib.keyboard_ctrl import KEY_COMMANDS
    steer = float(KEY_COMMANDS["a"].split("_")[1])
    torque = steer * SimulatorBridge.MANUAL_TORQUE
    self.assertGreater(torque, 1200)  # Honda Civic STEER_THRESHOLD default
    self.assertLess(float(KEY_COMMANDS["d"].split("_")[1]) * SimulatorBridge.MANUAL_TORQUE, -1200)


class TestCruiseButtonRelease(unittest.TestCase):
  def play(self, last_pressed, idle, button, frames):
    sent, held = [], frames
    while held > 0:
      out, held = cruise_button_step(last_pressed, idle, button, held)
      sent.append(out)
      if out:
        last_pressed, idle = out, 0
      else:
        idle += 1
    return sent

  def test_button_after_idle_is_sent_at_once(self):
    self.assertEqual(self.play(0, 0, 4, 3), [4, 4, 4])

  def test_different_button_right_after_another_waits_for_the_release(self):
    # 3 -> 4 straight would never report the release of 3 and leave controlsd's decelCruise timer running
    sent = self.play(3, 0, 4, 5)
    self.assertEqual(sent, [0] * CRUISE_RELEASE_FRAMES + [4] * 5)

  def test_a_one_frame_gap_is_not_enough(self):
    # the autoengage pulse ends, one frame of none, then the cruise ramp: controlsd can read both frames in one batch
    self.assertEqual(self.play(3, 1, 4, 2), [0] * (CRUISE_RELEASE_FRAMES - 1) + [4, 4])

  def test_enough_idle_frames_need_no_wait(self):
    self.assertEqual(self.play(3, CRUISE_RELEASE_FRAMES, 4, 2), [4, 4])

  def test_same_button_needs_no_release(self):
    self.assertEqual(self.play(4, 0, 4, 2), [4, 4])

  def test_waiting_for_the_release_does_not_shorten_a_long_press(self):
    self.assertEqual(self.play(3, 0, 4, 55).count(4), 55)


class TestLaneChangeSpeedGate(unittest.TestCase):
  MPH_30 = 30 * 0.44704

  def test_simulator_override_is_in_kph(self):
    self.assertAlmostEqual(lane_change_speed_min("30"), 30 / 3.6)

  def test_unset_or_invalid_keeps_30_mph(self):
    for value in ("", "x", "0", "-5"):
      self.assertAlmostEqual(lane_change_speed_min(value), self.MPH_30, places=4)


if __name__ == "__main__":
  unittest.main()
