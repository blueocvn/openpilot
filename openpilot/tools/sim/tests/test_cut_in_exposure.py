import unittest
from openpilot.tools.sim.cut_in_exposure import ExposureCollector


class TestExposureCollector(unittest.TestCase):
  @staticmethod
  def sample(actor_id, *, stale=False, actors=True):
    now = 2_000_000_000
    source_time = now - (300_000_000 if stale else 50_000_000)
    return {"type": "control_sample", "scene_time_s": 2.1, "host_monotonic_ns": now,
            "actors": ([{"id": actor_id, "ego_forward_gap_m": 7.0, "ego_lateral_offset_m": .1,
                         "bumper_gap_m": 5.0}] if actors else []),
            "openpilot": {"snapshot_monotonic_ns": source_time,
                           "message_status": {service: {"valid": True, "alive": True, "mono_time_ns": source_time}
                                              for service in ("modelV2", "radarState")}}}
  def test_completed_exposure_keeps_measured_values(self):
    collector = ExposureCollector()
    for record in (
      {"type": "cut_in_started", "actor_id": 7, "direction": "right_to_left", "scene_time_s": 1.0,
       "spawn_footprints_clear": True, "front_of_ego_teleport": False},
      {"type": "cut_in_entered_ego_lane", "actor_id": 7, "actual_gap_m": 7.1,
       "ego_speed_mps": 8.2, "bike_speed_mps": 5.0, "scene_time_s": 2.0},
      {"type": "cut_in_left_ego_lane", "actor_id": 7, "actual_hold_s": 1.7, "scene_time_s": 3.7},
      self.sample(7),
      {"type": "cut_in_finished", "actor_id": 7, "direction": "right_to_left", "scene_time_s": 4.0},
    ):
      collector.add(record)
    event = collector.finish()[0]
    self.assertEqual((event["actor_id"], event["direction"], event["entry_gap_m"], event["hold_s"]),
                     (7, "right_to_left", 7.1, 1.7))
    self.assertEqual((event["ego_speed_at_entry_mps"], event["bike_speed_at_entry_mps"]), (8.2, 5.0))
    self.assertTrue(event["complete"])
    self.assertFalse(event["evidence_missing"])

  def test_collision_is_kept_and_events_stay_separate(self):
    collector = ExposureCollector()
    collector.add({"type": "cut_in_started", "actor_id": 7, "direction": "left_to_right", "scene_time_s": 1.0,
                   "spawn_footprints_clear": True, "front_of_ego_teleport": False})
    collector.add({"type": "collision", "other_actor_id": 7, "scene_time_s": 1.5})
    collector.add({"type": "cut_in_finished", "actor_id": 7, "direction": "left_to_right", "scene_time_s": 2.0})
    collector.add({"type": "cut_in_started", "actor_id": 8, "direction": "right_to_left", "scene_time_s": 3.0,
                   "spawn_footprints_clear": True, "front_of_ego_teleport": False})
    events = collector.finish()
    self.assertTrue(events[0]["contact"])
    self.assertFalse(events[1]["contact"])
    self.assertEqual([event["direction"] for event in events], ["left_to_right", "right_to_left"])

  def test_missing_entry_or_stale_snapshot_marks_evidence_missing(self):
    collector = ExposureCollector()
    collector.add({"type": "cut_in_started", "actor_id": 7, "direction": "right_to_left", "scene_time_s": 1.0,
                   "spawn_footprints_clear": True, "front_of_ego_teleport": False})
    collector.add(self.sample(7, stale=True))
    collector.add({"type": "cut_in_finished", "actor_id": 7, "direction": "right_to_left", "scene_time_s": 2.0})
    event = collector.finish()[0]
    self.assertTrue(event["evidence_missing"])
    self.assertIsNone(event["entry_gap_m"])

  def test_stale_snapshot_invalidates_anotherwise_complete_exposure(self):
    collector = ExposureCollector()
    collector.add({"type": "cut_in_started", "actor_id": 9, "direction": "left_to_right", "scene_time_s": 1,
                   "spawn_footprints_clear": True, "front_of_ego_teleport": False})
    collector.add({"type": "cut_in_entered_ego_lane", "actor_id": 9, "actual_gap_m": 7,
                   "ego_speed_mps": 8, "bike_speed_mps": 5, "scene_time_s": 2})
    collector.add({"type": "cut_in_left_ego_lane", "actor_id": 9, "actual_hold_s": 1.7, "scene_time_s": 3.7})
    collector.add(self.sample(9, stale=True))
    collector.add({"type": "cut_in_finished", "actor_id": 9, "direction": "left_to_right", "scene_time_s": 4})
    self.assertTrue(collector.finish()[0]["evidence_missing"])

  def test_finished_events_are_bounded(self):
    collector = ExposureCollector()
    for actor_id in range(100):
      collector.add({"type": "cut_in_started", "actor_id": actor_id, "direction": "right_to_left", "scene_time_s": actor_id})
      collector.add({"type": "cut_in_finished", "actor_id": actor_id, "direction": "right_to_left", "scene_time_s": actor_id + .1})
    self.assertEqual(collector.active_count, 0)
