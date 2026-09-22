import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.tools.sim.analyze_motorcycle_weave import iter_report_records, match_radar_lead_actor, summarize_records
from openpilot.tools.sim.bridge.carla.carla_world import startup_steering_angle
from openpilot.tools.sim.bridge.carla.carla_bridge import scene_cruise_speed
from openpilot.tools.sim.bridge.carla.scenes import motorcycle_weave as weave
from openpilot.tools.sim.bridge.carla.scenes.motorcycle_weave import (
  DEFAULT_EGO_SPEED_MPS,
  DEFAULT_MOTORCYCLE_SPEED_MPS,
  build_weave_events,
  event_progress,
  finite_or_none,
  find_middle_driving_lane,
  lookahead_event_progress,
  motorcycle_longitudinal_control,
  motorcycle_steer_command,
  polygon_clearance,
  quintic_progress,
  relocated_spawn_height,
  scenario_ready,
  same_lane_path,
  tracking_error_components,
  trajectory_distance,
  MotorcycleWeaveScene,
  ContinuousWeaveScheduler,
  pass_lateral_progress,
  advance_path_distance,
  sha256_path,
  param_value_text,
  background_cut_in_ready,
  ego_relative_longitudinal,
  should_retire_completed,
)
from openpilot.tools.sim.bridge.carla.scenes.report import RotatingReport
from openpilot.tools.sim.run_bridge import parse_args, should_poll_keyboard


class TestMotorcycleWeaveSchedule(unittest.TestCase):
  def test_actor_is_reusable_only_after_ego_has_passed_it(self):
    self.assertAlmostEqual(ego_relative_longitudinal(10.0, 20.0, 0.0, 0.0, 0.0), -10.0)
    self.assertAlmostEqual(ego_relative_longitudinal(10.0, 20.0, 0.0, 0.0, 180.0), 10.0)

  def test_infinite_scene_does_not_end_when_elapsed_exceeds_default_duration(self):
    ego = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=8.33, y=0.0, z=0.0),
                          bounding_box=SimpleNamespace(extent=SimpleNamespace(x=2.4)))
    scene = MotorcycleWeaveScene(object(), object(), ego, [], duration_s=0)
    scene._prepare = lambda: None
    scene._candidate_lanes = lambda: (object(), object(), object(), 100.0)
    scene._maintain_background = lambda *args: None
    scene._start_pass = lambda event, *_: True
    scene.set_openpilot_ready(10.0)
    scene.latest_openpilot = {"active": True, "long_active": True}
    scene.before_tick(10.0)
    scene.before_tick(100.0)
    self.assertFalse(scene.finished)
    self.assertEqual(scene.scheduler.event_index, 0)  # no warmed adjacent motorcycle is available

  def test_finite_scene_still_ends_without_scheduling_after_duration(self):
    ego = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=8.33, y=0.0, z=0.0),
                          bounding_box=SimpleNamespace(extent=SimpleNamespace(x=2.4)))
    scene = MotorcycleWeaveScene(object(), object(), ego, [], duration_s=5.0)
    scene._prepare = lambda: None
    scene._candidate_lanes = lambda: (object(), object(), object(), 100.0)
    scene._maintain_background = lambda *args: None
    scene._start_pass = lambda event, *_: True
    scene.set_openpilot_ready(10.0)
    scene.latest_openpilot = {"active": True, "long_active": True}
    scene.before_tick(10.0)
    scene.before_tick(16.0)
    self.assertTrue(scene.finished)
    self.assertEqual(scene.scheduler.event_index, 0)

  def test_ground_truth_writes_at_most_two_samples_per_simulated_second(self):
    class Velocity:
      x = y = z = 0.0

    class Ego:
      def get_transform(self):
        return object()

      def get_velocity(self):
        return Velocity()

      def get_control(self):
        return SimpleNamespace(brake=0.35, throttle=0.0)

    scene = MotorcycleWeaveScene(object(), object(), Ego(), [], duration_s=0)
    scene.start_time_s = 0.0
    records = []
    scene._write = records.append
    for tick in (0.0, 0.1, 0.49, 0.5, 0.99, 1.0):
      scene.after_tick(tick)
    ground_truth = [record for record in records if record["type"] == "ground_truth"]
    self.assertEqual([record["scene_time_s"] for record in ground_truth], [0.0, 0.5, 1.0])
    self.assertEqual([record["ego_brake"] for record in ground_truth], [0.35] * 3)

  def test_short_brake_is_recorded_at_ten_hz_between_ground_truth_samples(self):
    class Ego:
      def get_transform(self):
        return object()

      def get_velocity(self):
        return SimpleNamespace(x=8.0, y=0.0, z=0.0)

      def get_control(self):
        return SimpleNamespace(brake=self.brake, throttle=0.0)

    ego = Ego()
    ego.brake = 0.0
    scene = MotorcycleWeaveScene(object(), object(), ego, [], duration_s=0)
    scene.start_time_s = 0.0
    scene.latest_openpilot = {"plan_has_lead": True, "lead_distance_m": 7.0,
                              "requested_accel_mps2": -1.5}
    records = []
    scene._write = records.append
    for tick in (0.0, 0.05, 0.1, 0.2, 0.5):
      ego.brake = 0.12 if tick == 0.2 else 0.0
      scene.after_tick(tick)
    controls = [r for r in records if r["type"] == "control_sample"]
    self.assertEqual([r["scene_time_s"] for r in controls], [0.0, 0.1, 0.2, 0.5])
    self.assertEqual([r["scene_time_s"] for r in records if r["type"] == "ground_truth"], [0.0, 0.5])
    self.assertEqual(controls[2]["ego_brake"], 0.12)

  def test_streaming_summary_attributes_brake_to_a_single_cut_in(self):
    records = iter([
      {"type": "ground_truth", "scene_time_s": 0.0, "ego_speed_mps": 8.3, "actors": []},
      {"type": "cut_in_started", "scene_time_s": 1.0, "actor_id": 7},
      {"type": "control_sample", "scene_time_s": 1.1, "ego_speed_mps": 8.2, "ego_brake": 0.0,
       "openpilot": {"plan_has_lead": False, "requested_accel_mps2": 0.1}},
      {"type": "control_sample", "scene_time_s": 1.2, "ego_speed_mps": 8.0, "ego_brake": 0.16,
       "openpilot": {"plan_has_lead": True, "requested_accel_mps2": -2.0}},
      {"type": "cut_in_finished", "scene_time_s": 1.3, "actor_id": 7},
      {"type": "ground_truth", "scene_time_s": 1.5, "ego_speed_mps": 7.8, "actors": []},
    ])
    summary = summarize_records(records)
    self.assertEqual(summary["control_sample_count"], 2)
    self.assertEqual(summary["braking_sample_count"], 1)
    self.assertEqual(summary["cut_ins_with_lead_count"], 1)
    self.assertEqual(summary["cut_ins_with_brake_count"], 1)
    self.assertEqual(summary["minimum_requested_accel_mps2"], -2.0)

  def test_radar_lead_actor_match_requires_unique_geometry(self):
    lead = {"present": True, "distance_m": 7.0, "lateral_m": 0.2}
    bike = {"id": 42, "ego_forward_gap_m": 7.3, "ego_lateral_offset_m": 0.1}
    self.assertEqual(match_radar_lead_actor(lead, [bike]), 42)
    self.assertIsNone(match_radar_lead_actor(lead, [bike, {"id": 43, "ego_forward_gap_m": 7.1,
                                                         "ego_lateral_offset_m": 0.2}]))
    self.assertIsNone(match_radar_lead_actor({"present": False, "distance_m": 7.0}, [bike]))

  def test_tracking_gate_ignores_samples_after_first_contact(self):
    records = [
      {"type": "cut_in_started", "scene_time_s": 0.0, "actor_id": 42},
      {"type": "ground_truth", "scene_time_s": 0.5, "ego_speed_mps": 8.0,
       "actors": [{"id": 42, "role": "cut_in", "lateral_tracking_error_m": 0.2,
                   "speed_mps": 5.0, "target_speed_mps": 5.0}]},
      {"type": "collision", "scene_time_s": 0.8, "other_actor_id": 42,
       "other_actor_type": "vehicle.vespa.zx125"},
      {"type": "ground_truth", "scene_time_s": 1.0, "ego_speed_mps": 4.0,
       "actors": [{"id": 42, "role": "cut_in", "lateral_tracking_error_m": 2.0,
                   "speed_mps": 1.0, "target_speed_mps": 5.0}]},
    ]
    summary = summarize_records(iter(records))
    self.assertEqual(summary["p95_lateral_tracking_error_m"], 0.2)
    self.assertEqual(summary["tracking_samples_after_contact_excluded"], 1)

  def test_summary_counts_lead_and_real_braking_samples(self):
    records = [
      {"type": "cut_in_started", "scene_time_s": 0.0},
      {"type": "ground_truth", "scene_time_s": 0.0, "ego_speed_mps": 8.0,
       "ego_brake": 0.0, "actors": [], "collisions": [], "openpilot": {"plan_has_lead": False}},
      {"type": "ground_truth", "scene_time_s": 0.5, "ego_speed_mps": 7.8,
       "ego_brake": 0.3, "actors": [], "collisions": [],
       "openpilot": {"plan_has_lead": True, "requested_accel_mps2": -0.8}},
    ]
    summary = summarize_records(iter(records))
    self.assertEqual(summary["lead_sample_count"], 1)
    self.assertEqual(summary["braking_sample_count"], 1)
    self.assertEqual(summary["maximum_ego_brake"], 0.3)

  def test_zero_duration_keeps_scene_open_and_negative_duration_is_rejected(self):
    args = parse_args(["--carla-scene-duration", "0"])
    self.assertEqual(args.carla_scene_duration, 0.0)
    with self.assertRaises(SystemExit):
      parse_args(["--carla-scene-duration", "-1"])

  def test_seeded_scheduler_repeats_directions_and_stays_inside_requested_ranges(self):
    first = ContinuousWeaveScheduler("alternating", seed=42)
    second = ContinuousWeaveScheduler("alternating", seed=42)
    events = []
    now = 0.0
    for _ in range(8):
      event = first.propose(now, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
      self.assertIsNotNone(event)
      first.commit(event)
      events.append(event)
      now = event.next_start_time_s
    replay = []
    now = 0.0
    for _ in range(8):
      event = second.propose(now, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
      second.commit(event)
      replay.append(event)
      now = event.next_start_time_s
    self.assertEqual(events, replay)
    self.assertEqual([event.direction for event in events], ["right_to_left", "left_to_right"] * 4)
    self.assertTrue(all(2.0 <= event.interval_s <= 3.0 for event in events))
    self.assertTrue(all(4.5 <= event.speed_mps <= 5.5 for event in events))
    self.assertTrue(all(6.0 <= event.merge_gap_m <= 8.0 for event in events))
    self.assertTrue(all(0.75 <= event.hold_s <= 1.05 for event in events))
    self.assertTrue(all(event.merge_gap_m - (8.33 - event.speed_mps) * event.hold_s < 5.0
                        for event in events))

  def test_scene_cruise_and_ready_gate_are_30_kmh(self):
    self.assertAlmostEqual(scene_cruise_speed("motorcycle_weave", stock_speed_mps=8.0), 30.0 / 3.6)
    self.assertFalse(scenario_ready(active=True, long_active=True, ego_speed_mps=7.49))
    self.assertFalse(scenario_ready(active=True, long_active=False, ego_speed_mps=8.33))
    self.assertTrue(scenario_ready(active=True, long_active=True, ego_speed_mps=7.5))

  def test_spawn_uses_oriented_footprints_and_rejects_overlap(self):
    candidate = weave.proposed_motorcycle_footprint(0.0, 0.0, 0.0)
    ego = [(-2.4, -1), (2.4, -1), (2.4, 1), (-2.4, 1)]
    self.assertFalse(weave.spawn_footprints_clear(candidate, ego, [], ego_margin_m=4.0))
    candidate = weave.proposed_motorcycle_footprint(18.0, 0.0, 0.0)
    nearby = weave.proposed_motorcycle_footprint(23.0, 0.0, 0.0)
    self.assertFalse(weave.spawn_footprints_clear(candidate, ego, [nearby], actor_margin_m=6.0))
    far = weave.proposed_motorcycle_footprint(40.0, 0.0, 0.0)
    self.assertTrue(weave.spawn_footprints_clear(far, ego, [nearby]))

  def test_seeded_background_flow_stays_bounded_on_both_sides(self):
    first = weave.BackgroundTrafficScheduler(seed=42)
    second = weave.BackgroundTrafficScheduler(seed=42)
    self.assertEqual(first.target_count, second.target_count)
    self.assertIn(first.target_count, (3, 4, 5))
    slots = []
    replay = []
    for _ in range(12):
      slots.append(first.next_slot())
      first.commit()
      replay.append(second.next_slot())
      second.commit()
    self.assertEqual(slots, replay)
    self.assertEqual([slot.side for slot in slots[:4]], ["left", "right", "left", "right"])
    self.assertTrue(all(5.0 <= slot.speed_mps <= 7.0 for slot in slots))
    self.assertTrue(all(25.0 <= slot.distance_m <= 43.0 for slot in slots))

  def test_background_bike_is_promoted_only_at_stable_speed_and_safe_gap(self):
    event = SimpleNamespace(speed_mps=5.0, merge_gap_m=7.0, entry_s=1.3)
    required = 7.0 + 3.5 + (8.3 - 5.2) * 1.3 + 1.2
    self.assertTrue(background_cut_in_ready(event, distance_m=required, ego_speed_mps=8.3,
                                            bike_speed_mps=5.2, vehicle_length_buffer_m=3.5))
    self.assertFalse(background_cut_in_ready(event, distance_m=required, ego_speed_mps=8.3,
                                             bike_speed_mps=3.0, vehicle_length_buffer_m=3.5))
    self.assertFalse(background_cut_in_ready(event, distance_m=10.0, ego_speed_mps=8.3,
                                             bike_speed_mps=5.2, vehicle_length_buffer_m=3.5))

  def test_cut_in_promotes_a_moving_background_actor_without_teleport(self):
    class Lane:
      road_id = 1
      lane_type = "driving"

      def __init__(self, lane_id):
        self.lane_id = lane_id

      def get_left_lane(self):
        return Lane(self.lane_id + 1)

      def next(self, _step):
        return [Lane(self.lane_id)]

    class Actor:
      id = 42

      def get_location(self):
        return SimpleNamespace(x=15.0, y=3.5)

      def set_transform(self, _transform):
        raise AssertionError("a visible bike was teleported")

    actor = Actor()
    world = SimpleNamespace(get_map=lambda: SimpleNamespace(get_waypoint=lambda *_args, **_kwargs: Lane(1)))
    scene = MotorcycleWeaveScene(SimpleNamespace(LaneType=SimpleNamespace(Driving="driving")),
                                 world, object(), [actor])
    item = {"actor": actor, "side": "right", "speed_mps": 5.0}
    scene._background.append(item)
    records = []
    scene._write = records.append
    event = scene.scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    self.assertTrue(scene._start_pass(event, Lane(2), Lane(3), Lane(1), item))
    self.assertEqual(scene._background, [])
    self.assertIs(scene._actors[0]["actor"], actor)
    self.assertTrue(records[0]["from_staged_background"])

  def test_background_attempt_does_not_require_hazard_to_be_finished(self):
    ego = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=8.33, y=0.0, z=0.0),
                          bounding_box=SimpleNamespace(extent=SimpleNamespace(x=2.4)))
    scene = MotorcycleWeaveScene(object(), object(), ego, [], duration_s=0)
    scene._prepare = lambda: None
    scene._candidate_lanes = lambda: (object(), object(), object(), 80.0)
    scene._actors = [{"completed": False, "actor": SimpleNamespace(is_alive=False)}]
    maintained = []
    scene._maintain_background = lambda *args: maintained.append(args)
    scene.set_openpilot_ready(10.0)
    scene.latest_openpilot = {"active": True, "long_active": True}
    scene.before_tick(10.0)
    self.assertEqual(len(maintained), 1)

  def test_background_actor_count_is_capped_and_spawned_on_adjacent_lanes(self):
    class Box:
      extent = SimpleNamespace(x=1.1)

      def get_world_vertices(self, transform):
        x, y = transform.location.x, transform.location.y
        return [SimpleNamespace(x=x + dx, y=y + dy) for dx, dy in
                ((-1.1, -0.5), (1.1, -0.5), (1.1, 0.5), (-1.1, 0.5))]

    class Transform:
      def __init__(self, x, y):
        self.location = SimpleNamespace(x=x, y=y, z=0.0)
        self.rotation = SimpleNamespace(yaw=0.0)

      def get_forward_vector(self):
        return SimpleNamespace(x=1.0, y=0.0)

    class Lane:
      road_id = 1
      lane_type = "driving"

      def __init__(self, x, y, lane_id):
        self.transform = Transform(x, y)
        self.lane_id = lane_id

      def next(self, step):
        return [Lane(self.transform.location.x + step, self.transform.location.y, self.lane_id)]

    class Actor:
      is_alive = True
      bounding_box = Box()

      def __init__(self, transform, actor_id):
        self.transform = transform
        self.id = actor_id

      def get_transform(self):
        return self.transform

      def set_target_velocity(self, velocity):
        self.velocity = velocity

    spawned = []
    library = SimpleNamespace(find=lambda name: name)
    world = SimpleNamespace(get_blueprint_library=lambda: library)
    world.try_spawn_actor = lambda blueprint, transform: spawned.append(Actor(transform, len(spawned) + 1)) or spawned[-1]
    ego = SimpleNamespace(get_transform=lambda: Transform(0.0, 0.0), bounding_box=Box())
    carla = SimpleNamespace(Vector3D=SimpleNamespace)
    scene = MotorcycleWeaveScene(carla, world, ego, [], seed=42)
    for _ in range(8):
      scene._maintain_background(0.0, Lane(0.0, -3.5, 1), Lane(0.0, 3.5, 3))
    self.assertEqual(len(scene._background), scene.background_scheduler.target_count)
    self.assertTrue(3 <= len(spawned) <= 5)
    self.assertLess(spawned[0].transform.location.y, 0.0)
    self.assertGreater(spawned[1].transform.location.y, 0.0)

  def test_scheduler_defers_unsafe_events_without_consuming_seed_or_direction(self):
    scheduler = ContinuousWeaveScheduler("alternating", seed=42)
    self.assertIsNone(scheduler.propose(0.0, ego_speed_mps=3.0, available_road_m=100.0, actor_available=True))
    self.assertIsNone(scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=12.0, actor_available=True))
    self.assertIsNone(scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=False))
    event = scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    self.assertEqual(event.direction, "right_to_left")
    self.assertGreater(event.spawn_center_distance_m, event.merge_gap_m)

  def test_failed_carla_spawn_does_not_consume_next_event(self):
    scheduler = ContinuousWeaveScheduler("alternating", seed=42)
    first_attempt = scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    retried = scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    self.assertEqual(first_attempt, retried)
    scheduler.commit(retried)
    self.assertIsNone(scheduler.propose(1.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True))

  def test_following_events_continue_after_first_lead_slows_ego(self):
    scheduler = ContinuousWeaveScheduler("alternating", seed=42)
    first = scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    scheduler.commit(first)
    self.assertIsNone(scheduler.propose(first.next_start_time_s, ego_speed_mps=4.8,
                                        available_road_m=100.0, actor_available=True))
    second = scheduler.propose(first.next_start_time_s, ego_speed_mps=8.0,
                               available_road_m=100.0, actor_available=True)
    self.assertIsNotNone(second)
    self.assertEqual(second.direction, "left_to_right")

  def test_completed_bike_retires_only_after_leaving_ego_lane(self):
    self.assertFalse(should_retire_completed(pass_elapsed_s=4.8, total_s=5.0, lateral_error_m=0.1))
    self.assertFalse(should_retire_completed(pass_elapsed_s=5.3, total_s=5.0, lateral_error_m=2.0))
    self.assertTrue(should_retire_completed(pass_elapsed_s=5.6, total_s=5.0, lateral_error_m=0.5))
    self.assertTrue(should_retire_completed(pass_elapsed_s=7.1, total_s=5.0, lateral_error_m=2.0))

  def test_motorcycle_controller_does_not_force_velocity_each_tick(self):
    class Actor:
      def __init__(self):
        self.target_velocity = None

      def get_transform(self):
        return SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0),
                               rotation=SimpleNamespace(yaw=0.0),
                               get_forward_vector=lambda: SimpleNamespace(x=1.0, y=0.0))

      def get_velocity(self):
        return SimpleNamespace(x=3.0, y=0.0, z=0.0)

      def apply_control(self, _control):
        pass

      def set_target_velocity(self, velocity):
        self.target_velocity = velocity

    carla = SimpleNamespace(VehicleControl=SimpleNamespace, Vector3D=SimpleNamespace)
    scene = MotorcycleWeaveScene(carla, object(), object(), [])
    event = scene.scheduler.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    actor = Actor()
    scene._apply_control({"actor": actor, "event": event, "speed_integral": 0.0},
                         SimpleNamespace(x=8.0, y=0.0))
    self.assertIsNone(actor.target_velocity)

  def test_retired_actor_is_removed_from_world_cleanup_list(self):
    class Actor:
      id = 123

      def __init__(self):
        self.destroyed = False

      def destroy(self):
        self.destroyed = True

    actor = Actor()
    scene = MotorcycleWeaveScene(object(), object(), object(), [actor])
    item = {"actor": actor}
    scene._actors.append(item)
    records = []
    scene._write = records.append
    scene._retire_actor(item, 7.0)
    self.assertTrue(actor.destroyed)
    self.assertEqual(scene._actors, [])
    self.assertEqual(scene.actor_sink, [])
    self.assertEqual(records[0]["type"], "cut_in_retired")

  def test_collision_callbacks_do_not_grow_scene_memory_forever(self):
    scene = MotorcycleWeaveScene(object(), object(), object(), [])
    for actor_id in range(100):
      scene._on_collision(SimpleNamespace(other_actor=SimpleNamespace(id=actor_id, type_id="vespa")))
    self.assertLessEqual(len(scene._collisions), 32)

  def test_collision_callback_writes_timestamp_frame_and_impulse_immediately(self):
    scene = MotorcycleWeaveScene(object(), object(), object(), [])
    scene.start_time_s = 10.0
    records = []
    scene._write = records.append
    scene._on_collision(SimpleNamespace(
      timestamp=12.5, frame=123,
      other_actor=SimpleNamespace(id=42, type_id="vehicle.vespa.zx125"),
      normal_impulse=SimpleNamespace(x=3.0, y=4.0, z=0.0)))
    self.assertEqual(records[0]["type"], "collision")
    self.assertEqual(records[0]["scene_time_s"], 2.5)
    self.assertEqual(records[0]["frame"], 123)
    self.assertEqual(records[0]["other_actor_id"], 42)
    self.assertEqual(records[0]["impulse_norm"], 5.0)

  def test_two_runs_started_same_second_never_overwrite_a_report(self):
    with TemporaryDirectory() as directory, \
         patch("openpilot.tools.sim.bridge.carla.scenes.motorcycle_weave.time.strftime", return_value="20260922-020000"):
      first = MotorcycleWeaveScene(object(), object(), object(), [], report_dir=directory)
      second = MotorcycleWeaveScene(object(), object(), object(), [], report_dir=directory)
      self.assertNotEqual(first.report_dir, second.report_dir)

  def test_manifest_file_hash_is_content_based(self):
    with TemporaryDirectory() as directory:
      path = Path(directory) / "artifact.bin"
      path.write_bytes(b"abc")
      self.assertEqual(sha256_path(path),
                       "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")

  def test_manifest_accepts_typed_params_as_well_as_raw_bytes(self):
    self.assertEqual(param_value_text(True), "True")
    self.assertEqual(param_value_text(b"2"), "2")

  def test_scene_start_does_not_hash_artifacts_on_simulator_tick(self):
    sensor = SimpleNamespace(listen=lambda _callback: None)
    world = SimpleNamespace(get_blueprint_library=lambda: SimpleNamespace(find=lambda _name: object()),
                            spawn_actor=lambda *_args, **_kwargs: sensor)
    with TemporaryDirectory() as directory:
      scene = MotorcycleWeaveScene(SimpleNamespace(Transform=SimpleNamespace), world, object(), [],
                                   report_dir=directory)
      scene._write_manifest = Mock(side_effect=AssertionError("manifest hash blocked CARLA tick"))
      scene._prepare()
      self.assertFalse(scene._write_manifest.called)
      scene.close()

  def test_motorcycle_enters_holds_then_exits_ego_lane(self):
    event = ContinuousWeaveScheduler("alternating", seed=42).propose(
      0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
    self.assertAlmostEqual(pass_lateral_progress(event, event.entry_s), 0.5)
    self.assertAlmostEqual(pass_lateral_progress(event, event.entry_s + event.hold_s / 2), 0.5)
    self.assertAlmostEqual(pass_lateral_progress(event, event.entry_s + event.hold_s), 0.5)
    self.assertAlmostEqual(pass_lateral_progress(event, event.total_s), 1.0)

  def test_path_progress_follows_measured_forward_motion_not_scene_time(self):
    self.assertAlmostEqual(advance_path_distance(2.0, velocity_x=3.0, velocity_y=4.0,
                                                 route_yaw_deg=0.0, dt_s=0.2), 2.6)
    self.assertAlmostEqual(advance_path_distance(2.0, velocity_x=-3.0, velocity_y=0.0,
                                                 route_yaw_deg=0.0, dt_s=0.2), 2.0)

  def test_reference_pose_does_not_advance_when_actor_is_stopped(self):
    scene = MotorcycleWeaveScene(SimpleNamespace(Location=SimpleNamespace), object(), object(), [])
    scene._advance = lambda lane, distance: SimpleNamespace(
      transform=SimpleNamespace(location=SimpleNamespace(x=distance, y=lane, z=0.0),
                                rotation=SimpleNamespace(yaw=0.0)))
    event = SimpleNamespace(spawn_center_distance_m=10.0)
    item = {"event": event, "source": 3.0, "middle": 0.0, "target": -3.0, "progress_m": 2.0}
    first, _ = scene._target_pose(item, 0.0, 0.5, lookahead_m=0.0)
    later, _ = scene._target_pose(item, 100.0, 0.5, lookahead_m=0.0)
    self.assertEqual((first.x, first.y), (12.0, 0.0))
    self.assertEqual((later.x, later.y), (first.x, first.y))

  def test_lateral_target_anticipates_a_fixed_distance_not_elapsed_time(self):
    scene = MotorcycleWeaveScene(object(), object(), object(), [])
    event = SimpleNamespace(speed_mps=5.0, entry_s=1.2, hold_s=1.8, exit_s=1.2)
    item = {"event": event, "progress_m": 5.0}
    expected = pass_lateral_progress(event, (5.0 + 3.0) / 5.0)
    self.assertAlmostEqual(scene._control_progress(item), expected)
    item["progress_m"] = 15.5
    self.assertAlmostEqual(scene._control_progress(item), pass_lateral_progress(event, (15.5 + 1.2) / 5.0))

  def test_lane_hold_is_measured_from_actor_position(self):
    scene = MotorcycleWeaveScene(SimpleNamespace(), object(), object(), [])
    scene._advance = lambda _lane, distance: SimpleNamespace(
      transform=SimpleNamespace(location=SimpleNamespace(x=distance, y=0.0, z=0.0),
                                rotation=SimpleNamespace(yaw=0.0)))
    actor = SimpleNamespace(id=42, get_location=lambda: SimpleNamespace(x=12.0, y=0.2))
    event = SimpleNamespace(spawn_center_distance_m=10.0, speed_mps=5.0)
    item = {"actor": actor, "event": event, "middle": object(), "progress_m": 2.0,
            "entered_ego_lane_at": None, "left_ego_lane_at": None}
    records = []
    scene._write = records.append
    scene._track_lane_occupancy(item, 1.0)
    self.assertEqual(item["entered_ego_lane_at"], 1.0)
    actor.get_location = lambda: SimpleNamespace(x=12.0, y=1.0)
    scene._track_lane_occupancy(item, 2.75)
    self.assertEqual(records[-1]["type"], "cut_in_left_ego_lane")
    self.assertEqual(records[-1]["actual_hold_s"], 1.75)

  def test_report_rotates_and_reader_streams_all_segments_in_order(self):
    with TemporaryDirectory() as directory:
      report_dir = Path(directory)
      with RotatingReport(report_dir, max_bytes=130) as report:
        for index in range(8):
          report.write({"type": "cut_in_started", "scene_time_s": float(index), "actor_id": index})
      self.assertGreater(len(list(report_dir.glob("events*.jsonl"))), 1)
      self.assertEqual([record["actor_id"] for record in iter_report_records(report_dir)], list(range(8)))

  def test_rotating_reader_keeps_numeric_order_beyond_four_digits(self):
    with TemporaryDirectory() as directory:
      root = Path(directory)
      (root / "events.jsonl").write_text('{"type":"event","actor_id":0}\n', encoding="utf-8")
      (root / "events-9999.jsonl").write_text('{"type":"event","actor_id":1}\n', encoding="utf-8")
      (root / "events-10000.jsonl").write_text('{"type":"event","actor_id":2}\n', encoding="utf-8")
      self.assertEqual([record["actor_id"] for record in iter_report_records(root)], [0, 1, 2])

  def test_summary_accepts_one_pass_record_stream_across_rotated_files(self):
    with TemporaryDirectory() as directory:
      with RotatingReport(Path(directory), max_bytes=260) as report:
        report.write({"type": "ground_truth", "scene_time_s": 0.0,
                      "ego_speed_mps": 0.0, "actors": [], "collisions": []})
        report.write({"type": "cut_in_started", "scene_time_s": 2.0})
        for index in range(20):
          report.write({"type": "ground_truth", "scene_time_s": 2.0 + index * 0.5,
                        "ego_speed_mps": 8.0 - index * 0.1, "actors": [], "collisions": [],
                        "openpilot": {"requested_accel_mps2": -0.4}})
      summary = summarize_records(iter_report_records(Path(directory)))
      self.assertEqual(summary["sample_count"], 21)
      self.assertEqual(summary["duration_s"], 11.5)
      self.assertEqual(summary["evaluation_start_s"], 2.0)
      self.assertEqual(summary["minimum_ego_speed_mps"], 6.1)
      self.assertEqual(summary["minimum_requested_accel_mps2"], -0.4)

  def test_startup_lane_follow_steering_handles_heading_and_wraparound(self):
    """Catches a delayed openpilot startup sending the rolling ego straight off a curved lane."""
    self.assertEqual(startup_steering_angle(current_yaw_deg=0.0, target_yaw_deg=5.0), -40.0)
    self.assertEqual(startup_steering_angle(current_yaw_deg=179.0, target_yaw_deg=-179.0), -16.0)
    self.assertEqual(startup_steering_angle(current_yaw_deg=0.0, target_yaw_deg=90.0), -120.0)

  def test_alternating_schedule_has_six_opposite_direction_crossings(self):
    """Catches a schedule that sends every motorcycle through the same side."""
    events = build_weave_events("alternating")

    self.assertEqual([event.direction for event in events], [
      "right_to_left", "left_to_right", "right_to_left",
      "left_to_right", "right_to_left", "left_to_right",
    ])
    self.assertEqual([event.start_time_s for event in events], [6.0, 10.0, 14.0, 18.0, 22.0, 26.0])
    self.assertEqual([event.duration_s for event in events], [4.0] * 6)
    self.assertEqual([round(event.initial_distance_m, 2) for event in events],
                     [30.67, 40.00, 49.33, 58.67, 68.00, 77.33])

  def test_quintic_progress_starts_and_ends_without_lateral_jerk(self):
    """Catches a lane-change profile that jumps at either endpoint."""
    self.assertEqual(quintic_progress(-1.0), 0.0)
    self.assertEqual(quintic_progress(0.0), 0.0)
    self.assertAlmostEqual(quintic_progress(0.5), 0.5)
    self.assertEqual(quintic_progress(1.0), 1.0)
    self.assertEqual(quintic_progress(2.0), 1.0)

  def test_default_relative_speed_is_the_documented_30_kph_ego_case(self):
    """Catches a default speed change that silently invalidates the baseline."""
    self.assertAlmostEqual(DEFAULT_EGO_SPEED_MPS, 30.0 / 3.6)
    self.assertEqual(DEFAULT_MOTORCYCLE_SPEED_MPS, 6.0)

  def test_carla_cli_preserves_a_named_scene_and_seed(self):
    """Catches a bridge invocation that silently falls back to an empty road."""
    args = parse_args([
      "--simulator", "carla", "--carla-scene", "motorcycle_weave",
      "--carla-scene-case", "alternating", "--carla-scene-seed", "42",
      "--no-experimental-mode",
    ])

    self.assertEqual(args.carla_scene, "motorcycle_weave")
    self.assertEqual(args.carla_scene_case, "alternating")
    self.assertEqual(args.carla_scene_seed, 42)
    self.assertFalse(args.experimental_mode)

  def test_event_progress_only_crosses_during_its_window(self):
    """Catches an actor beginning a cut-in before openpilot has a stable baseline."""
    event = build_weave_events("right_to_left")[0]

    self.assertEqual(event_progress(event, 5.99), 0.0)
    self.assertAlmostEqual(event_progress(event, 8.0), 0.5)
    self.assertEqual(event_progress(event, 10.0), 1.0)

  def test_route_continues_across_sections_of_the_same_road_lane(self):
    """Catches a long scenario aborting at an ordinary CARLA section boundary."""
    class Waypoint:
      road_id = 12
      lane_id = -2

      def __init__(self, section_id):
        self.section_id = section_id

    self.assertTrue(same_lane_path(Waypoint(3), Waypoint(4)))
    self.assertFalse(same_lane_path(Waypoint(3), type("OtherLane", (), {
      "road_id": 12, "section_id": 3, "lane_id": -3,
    })()))

  def test_report_summary_counts_a_sustained_stop_and_restart(self):
    """Catches a summary that mistakes a single noisy speed sample for stop/go."""
    records = [
      {"type": "ground_truth", "scene_time_s": 0.0, "ego_speed_mps": 8.0,
       "actors": [{"distance_to_ego_m": 15.0, "tracking_error_m": 0.1, "speed_mps": 6.0}], "collisions": [],
       "openpilot": {"requested_accel_mps2": 0.1}},
      {"type": "ground_truth", "scene_time_s": 1.0, "ego_speed_mps": 0.2,
       "actors": [{"distance_to_ego_m": 8.0, "tracking_error_m": 0.2, "speed_mps": 5.8}], "collisions": [],
       "openpilot": {"requested_accel_mps2": -1.2}},
      {"type": "ground_truth", "scene_time_s": 2.0, "ego_speed_mps": 0.1,
       "actors": [{"distance_to_ego_m": 7.0, "tracking_error_m": 0.3, "speed_mps": 6.4}], "collisions": [],
       "openpilot": {"requested_accel_mps2": -0.8}},
      {"type": "ground_truth", "scene_time_s": 3.0, "ego_speed_mps": 1.2,
       "actors": [{"distance_to_ego_m": 9.0, "tracking_error_m": 0.4, "speed_mps": 5.4}], "collisions": [],
       "openpilot": {"requested_accel_mps2": 0.4}},
      {"type": "ground_truth", "scene_time_s": 4.0, "ego_speed_mps": 1.3,
       "actors": [{"distance_to_ego_m": 11.0, "tracking_error_m": 0.5, "speed_mps": 6.8}], "collisions": [],
       "openpilot": {"requested_accel_mps2": 0.2}},
    ]

    summary = summarize_records(records)

    self.assertEqual(summary["stop_count"], 1)
    self.assertEqual(summary["restart_count"], 1)
    self.assertEqual(summary["minimum_ego_speed_mps"], 0.1)
    self.assertEqual(summary["minimum_actor_clearance_m"], 7.0)
    self.assertEqual(summary["minimum_requested_accel_mps2"], -1.2)
    self.assertEqual(summary["p95_actor_tracking_error_m"], 0.5)
    self.assertAlmostEqual(summary["p95_motorcycle_speed_error_mps"], 0.8)

  def test_non_finite_openpilot_values_are_json_safe(self):
    """Catches startup NaNs aborting an otherwise valid CARLA report."""
    self.assertIsNone(finite_or_none(float("nan")))
    self.assertIsNone(finite_or_none(float("inf")))
    self.assertEqual(finite_or_none(-1.25), -1.25)

  def test_openpilot_observer_records_radar_lead_and_message_freshness(self):
    scene = MotorcycleWeaveScene.__new__(MotorcycleWeaveScene)
    values = {
      "carState": SimpleNamespace(vEgo=8.0, aEgo=-0.3),
      "carControl": SimpleNamespace(longActive=True, actuators=SimpleNamespace(accel=-0.5)),
      "carOutput": SimpleNamespace(actuatorsOutput=SimpleNamespace(accel=-0.4)),
      "selfdriveState": SimpleNamespace(active=True),
      "longitudinalPlan": SimpleNamespace(longitudinalPlanSource="lead0", aTarget=-0.5,
                                           shouldStop=False, hasLead=True),
      "modelV2": SimpleNamespace(leadsV3=[SimpleNamespace(prob=0.8, x=[9.0], y=[0.2], v=[5.0])]),
      "radarState": SimpleNamespace(leadOne=SimpleNamespace(present=True, dRel=7.0, yRel=0.1,
                                                             vRel=-3.0, vLead=5.0, modelProb=0.8, radar=False),
                                    leadTwo=SimpleNamespace(present=False, dRel=0.0, yRel=0.0,
                                                             vRel=0.0, vLead=0.0, modelProb=0.0, radar=False)),
    }
    class FakeMaster(dict):
      valid = dict.fromkeys(values, True)
      alive = dict.fromkeys(values, True)
      logMonoTime = dict.fromkeys(values, 123456)
    scene.record_openpilot(FakeMaster(values))
    self.assertEqual(scene.latest_openpilot["radar_lead_one"]["distance_m"], 7.0)
    self.assertEqual(scene.latest_openpilot["message_status"]["radarState"]["mono_time_ns"], 123456)

  def test_missing_openpilot_services_do_not_abort_recording(self):
    """Catches startup telemetry gaps crashing the CARLA bridge."""
    scene = MotorcycleWeaveScene.__new__(MotorcycleWeaveScene)
    scene.latest_openpilot = {"available": True}

    scene.record_openpilot({})

    self.assertEqual(scene.latest_openpilot, {"available": False})

  def test_headless_bridge_does_not_start_a_terminal_keyboard_reader(self):
    """Catches non-interactive WSL runs failing in termios.tcgetattr()."""
    self.assertFalse(should_poll_keyboard(joystick=False, stdin_isatty=False))
    self.assertTrue(should_poll_keyboard(joystick=False, stdin_isatty=True))
    self.assertFalse(should_poll_keyboard(joystick=True, stdin_isatty=True))

  def test_scene_moves_from_edge_spawn_to_middle_driving_lane(self):
    """Catches Town04 spawn 40 placing ego beside a shoulder instead of two traffic lanes."""
    class Lane:
      road_id = 45
      lane_type = "driving"

      def __init__(self, lane_id):
        self.lane_id = lane_id
        self.left = None
        self.right = None

      def get_left_lane(self):
        return self.left

      def get_right_lane(self):
        return self.right

    edge, middle, outer = Lane(3), Lane(4), Lane(5)
    shoulder = Lane(2)
    shoulder.lane_type = "shoulder"
    edge.left, edge.right = shoulder, middle
    middle.left, middle.right = edge, outer
    outer.left = middle

    self.assertIs(find_middle_driving_lane(edge, "driving"), middle)

  def test_middle_lane_spawn_preserves_height_above_road(self):
    """Catches the ego bounding box intersecting the road after lateral relocation."""
    self.assertAlmostEqual(relocated_spawn_height(12.28, 12.0, 11.5), 11.78)

  def test_scene_waits_for_active_openpilot_at_baseline_speed(self):
    """Catches cut-in timing being anchored while the ego is still accelerating."""
    self.assertFalse(scenario_ready(active=False, long_active=True, ego_speed_mps=8.0))
    self.assertFalse(scenario_ready(active=True, long_active=True, ego_speed_mps=7.49))
    self.assertTrue(scenario_ready(active=True, long_active=True, ego_speed_mps=7.5))

  def test_motorcycle_scene_uses_safe_ego_cruise_without_changing_empty_scene(self):
    self.assertAlmostEqual(scene_cruise_speed("motorcycle_weave", stock_speed_mps=8.0), 30.0 / 3.6)
    self.assertEqual(scene_cruise_speed("none", stock_speed_mps=8.0), 8.0)

  def test_report_ignores_startup_stop_before_first_cut_in(self):
    """Catches initial standstill being reported as traffic-induced stop/go."""
    records = [
      {"type": "ground_truth", "scene_time_s": 0.0, "ego_speed_mps": 0.0, "actors": [], "collisions": []},
      {"type": "ground_truth", "scene_time_s": 1.0, "ego_speed_mps": 0.0, "actors": [], "collisions": []},
      {"type": "cut_in_started", "scene_time_s": 2.0},
      {"type": "ground_truth", "scene_time_s": 2.0, "ego_speed_mps": 8.0, "actors": [], "collisions": []},
      {"type": "ground_truth", "scene_time_s": 3.0, "ego_speed_mps": 8.0, "actors": [], "collisions": []},
    ]

    summary = summarize_records(records)

    self.assertEqual(summary["stop_count"], 0)
    self.assertEqual(summary["restart_count"], 0)

  def test_clearance_uses_vehicle_footprints_instead_of_bounding_circles(self):
    """Catches adjacent-lane vehicles being reported as overlapping when their boxes are separate."""
    ego = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    adjacent = [(3.0, -1.0), (5.0, -1.0), (5.0, 1.0), (3.0, 1.0)]
    overlapping = [(0.5, -1.0), (2.5, -1.0), (2.5, 1.0), (0.5, 1.0)]

    self.assertEqual(polygon_clearance(ego, adjacent), 2.0)
    self.assertEqual(polygon_clearance(ego, overlapping), 0.0)

  def test_tracking_reference_excludes_controller_lookahead(self):
    """Catches pure-pursuit lookahead being counted as physical tracking error."""
    self.assertEqual(trajectory_distance(30.0, 6.0, 2.0, lookahead_m=0.0), 42.0)
    self.assertEqual(trajectory_distance(30.0, 6.0, 2.0, lookahead_m=6.0), 48.0)

  def test_controller_looks_ahead_in_lateral_progress_as_well_as_distance(self):
    """Catches a controller target that is six metres ahead but laterally one second late."""
    event = build_weave_events("right_to_left")[0]

    self.assertEqual(lookahead_event_progress(event, 5.0, lookahead_m=6.0, speed_mps=6.0), 0.0)
    self.assertAlmostEqual(lookahead_event_progress(event, 6.0, lookahead_m=6.0, speed_mps=6.0),
                           quintic_progress(0.25))
    self.assertAlmostEqual(lookahead_event_progress(event, 8.0, lookahead_m=6.0, speed_mps=6.0),
                           quintic_progress(0.75))

  def test_tracking_error_is_split_in_the_reference_lane_frame(self):
    """Catches longitudinal lag being mislabeled as lane tracking error."""
    longitudinal, lateral = tracking_error_components(
      actual_x=7.0, actual_y=3.0, reference_x=10.0, reference_y=2.0, reference_yaw_deg=0.0,
    )

    self.assertAlmostEqual(longitudinal, -3.0)
    self.assertAlmostEqual(lateral, 1.0)

  def test_actor_tracking_gate_uses_lateral_error_not_longitudinal_lag(self):
    """Catches a stable lane trajectory failing because a motorcycle is slightly behind schedule."""
    records = [
      {"type": "ground_truth", "scene_time_s": 6.0, "ego_speed_mps": 8.0,
       "actors": [{"distance_to_ego_m": 10.0, "tracking_error_m": 3.0,
                   "longitudinal_tracking_error_m": -3.0, "lateral_tracking_error_m": 0.2,
                   "speed_mps": 6.0}], "collisions": []},
      {"type": "ground_truth", "scene_time_s": 7.0, "ego_speed_mps": 8.0,
       "actors": [{"distance_to_ego_m": 9.0, "tracking_error_m": 3.2,
                   "longitudinal_tracking_error_m": -3.2, "lateral_tracking_error_m": -0.3,
                   "speed_mps": 6.0}], "collisions": []},
    ]

    summary = summarize_records(records)

    self.assertEqual(summary["p95_actor_tracking_error_m"], 3.2)
    self.assertEqual(summary["p95_lateral_tracking_error_m"], 0.3)
    self.assertEqual(summary["maximum_lateral_tracking_error_m"], 0.3)
    self.assertTrue(summary["actor_tracking_valid"])

  def test_motorcycle_speed_controller_has_rolling_feedforward(self):
    """Catches motorcycles settling below 6 m/s due to road/engine resistance."""
    throttle, brake, integral = motorcycle_longitudinal_control(5.2, 0.2)
    self.assertGreaterEqual(throttle, 0.7)
    self.assertEqual(brake, 0.0)
    self.assertGreater(integral, 0.2)
    self.assertEqual(motorcycle_longitudinal_control(8.0, 0.2), (0.0, 0.4, 0.0))

  def test_motorcycle_steering_uses_full_range_for_large_lane_error(self):
    """Catches the Vespa understeering across two lanes during a four-second cut-in."""
    self.assertEqual(motorcycle_steer_command(local_x=6.0, local_y=6.0), 1.0)
    self.assertEqual(motorcycle_steer_command(local_x=6.0, local_y=-6.0), -1.0)
    self.assertEqual(motorcycle_steer_command(local_x=6.0, local_y=0.0), 0.0)
