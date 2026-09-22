import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from openpilot.tools.sim.analyze_motorcycle_weave import iter_report_records
from openpilot.tools.sim.bridge.carla.scenes.motorcycle_weave import MotorcycleWeaveScene
from openpilot.tools.sim.planner_trace import PlannerTraceRecorder, ReplayMaster, parity_report_valid, replay_mode_zero, trace_health
from openpilot.tools.sim.run_bridge import configure_scene_report_path
from openpilot.selfdrive.controls.plannerd import update_with_observer


class _Field:
  def __init__(self, value):
    self.value = value

  def to_dict(self):
    return {"vEgo": self.value}


class _Master:
  services = ("carState",)
  frame = 7
  updated = {"carState": True}
  alive = {"carState": True}
  valid = {"carState": True}
  freq_ok = {"carState": True}
  logMonoTime = {"carState": 123}

  def __getitem__(self, service):
    return _Field(8.3)

  def all_checks(self):
    return True


class TestPlannerTrace(unittest.TestCase):
  def test_replay_master_restores_capnp_values_and_status(self):
    snapshot = {"frame": 9, "all_checks": False, "data": {"carState": {"vEgo": 8.3}},
                "updated": {"carState": True}, "alive": {"carState": False},
                "valid": {"carState": True}, "freq_ok": {"carState": True},
                "log_mono_time_ns": {"carState": 123}}
    master = ReplayMaster(snapshot)
    self.assertAlmostEqual(master["carState"].vEgo, 8.3, places=5)
    self.assertEqual(master.logMonoTime["carState"], 123)
    self.assertFalse(master.all_checks())
    self.assertFalse(master.alive["carState"])

  def test_observer_failure_never_changes_planner_update(self):
    class Planner:
      def __init__(self):
        self.updated = False

      def update(self, sm):
        self.updated = sm is master

    class BrokenRecorder:
      def record(self, sm):
        raise OSError("disk full")

    master = _Master()
    planner = Planner()
    update_with_observer(planner, master, BrokenRecorder())
    self.assertTrue(planner.updated)

  def test_bridge_and_scene_share_one_report_directory(self):
    with TemporaryDirectory() as directory:
      environment = {}
      report = configure_scene_report_path(directory, environment, run_id="fixed-run")
      with patch.dict("os.environ", environment):
        scene = MotorcycleWeaveScene(None, None, None, [], report_dir=directory)
      self.assertEqual(report, scene.report_dir)
      self.assertEqual(report, Path(directory) / "motorcycle-weave-fixed-run")

  def test_records_exact_planner_input_and_rotates(self):
    with TemporaryDirectory() as directory:
      recorder = PlannerTraceRecorder(Path(directory), {"openpilotLongitudinalControl": True}, max_bytes=350)
      for index in range(3):
        master = _Master()
        master.frame = 7 + index
        recorder.record(master, tick_time_ns=1000 + index)
      recorder.close()
      records = list(iter_report_records(Path(directory) / "planner"))
      samples = [record for record in records if record["type"] == "planner_input"]
      self.assertEqual([sample["sequence"] for sample in samples], [0, 1, 2])
      self.assertEqual(samples[0]["data"]["carState"]["vEgo"], 8.3)
      self.assertEqual(samples[0]["log_mono_time_ns"]["carState"], 123)
      self.assertGreater(len(list((Path(directory) / "planner").glob("events*.jsonl"))), 1)
      health = trace_health(Path(directory))
      self.assertTrue(health["planner_trace_valid"])
      self.assertEqual(health["planner_trace_first_tick_ns"], 1000)
      self.assertEqual(health["planner_trace_last_tick_ns"], 1002)

  def test_missing_planner_tick_invalidates_trace(self):
    with TemporaryDirectory() as directory:
      recorder = PlannerTraceRecorder(Path(directory), {}, max_bytes=1000)
      recorder.record(_Master(), tick_time_ns=1000)
      recorder.sequence += 1
      master = _Master()
      master.frame = 8
      recorder.record(master, tick_time_ns=1001)
      recorder.close()
      result = trace_health(Path(directory))
      self.assertFalse(result["planner_trace_valid"])
      self.assertEqual(result["planner_trace_gap_count"], 1)

  def test_replay_mode_zero_matches_phase_one_planner_on_same_input(self):
    from opendbc.car.structs import car
    from openpilot.selfdrive.controls.tests.test_vn_traffic_planner import cruise_messages

    class Master:
      def __init__(self):
        self.data = cruise_messages()
        self.services = tuple(self.data)
        self.frame = 0
        self.updated = dict.fromkeys(self.services, True)
        self.alive = dict.fromkeys(self.services, True)
        self.valid = dict.fromkeys(self.services, True)
        self.freq_ok = dict.fromkeys(self.services, True)
        self.logMonoTime = dict.fromkeys(self.services, 1)

      def __getitem__(self, service):
        return self.data[service]

      def all_checks(self):
        return True

    cp = car.CarParams.new_message()
    cp.openpilotLongitudinalControl = True
    cp.longitudinalActuatorDelay = 0.2
    cp.steerRatio = 12.0
    cp.wheelbase = 2.9
    with TemporaryDirectory() as directory:
      recorder = PlannerTraceRecorder(Path(directory), cp)
      master = Master()
      recorder.record(master, tick_time_ns=1000)
      master.frame = 1
      recorder.record(master, tick_time_ns=1001)
      recorder.close()
      result = replay_mode_zero(Path(directory), "8ad43259c8ccbba082a15661a399c8dd0018194c")
      self.assertTrue(result["mode_zero_parity_verified"], result)
      self.assertEqual(result["ticks_compared"], 2)
      (Path(directory) / "planner-parity.json").write_text(json.dumps(result))
      self.assertTrue(parity_report_valid(Path(directory)))
      with (Path(directory) / "planner" / "events.jsonl").open("a") as output:
        output.write("\n")
      self.assertFalse(parity_report_valid(Path(directory)))


if __name__ == "__main__":
  unittest.main()
