from openpilot.tools.sim.analyze_radar_cutin import rotated_events
from openpilot.tools.sim.bridge.carla.scenes.report import RotatingReport


def test_radar_replay_reads_rotated_files_in_chronological_order(tmp_path):
  with RotatingReport(tmp_path, max_bytes=80) as report:
    for sequence in range(5):
      report.write({"type": "radar_frame", "sequence": sequence})
  assert len(list(tmp_path.glob("events*.jsonl"))) > 1
  assert [event["sequence"] for event in rotated_events(tmp_path)] == list(range(5))
