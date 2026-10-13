"""Simulation-only snapshots of the exact SubMaster state consumed by plannerd."""

import argparse
import base64
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from types import ModuleType
from unittest.mock import patch

from openpilot.tools.sim.analyze_motorcycle_weave import iter_report_records
from openpilot.tools.sim.bridge.carla.scenes.report import RotatingReport
from openpilot.tools.sim.lib.carla_radar import MAX_PAYLOAD_BYTES


RADAR_SERVICE = "customReservedRawData0"


def _serialize_service(sm, service):
  if service == RADAR_SERVICE:
    payload = bytes(sm[service])
    if len(payload) > MAX_PAYLOAD_BYTES:
      raise ValueError("radar trace payload too large")
    return {"encoding": "base64", "payload": base64.b64encode(payload).decode("ascii")}
  return sm[service].to_dict()


class PlannerTraceRecorder:
  def __init__(self, report_dir: Path, car_params, *, max_bytes: int = 10 * 1024 * 1024):
    self.report = RotatingReport(Path(report_dir) / "planner", max_bytes=max_bytes)
    self.sequence = 0
    params = car_params.to_dict() if hasattr(car_params, "to_dict") else car_params
    self.report.write({"type": "planner_header", "car_params": params})

  def record(self, sm, *, tick_time_ns: int | None = None):
    services = tuple(sm.services)
    self.report.write({
      "type": "planner_input", "sequence": self.sequence,
      "frame": sm.frame, "tick_time_ns": time.monotonic_ns() if tick_time_ns is None else tick_time_ns,
      "all_checks": bool(sm.all_checks()),
      "updated": {service: bool(sm.updated[service]) for service in services},
      "alive": {service: bool(sm.alive[service]) for service in services},
      "valid": {service: bool(sm.valid[service]) for service in services},
      "freq_ok": {service: bool(sm.freq_ok[service]) for service in services},
      "log_mono_time_ns": {service: int(sm.logMonoTime[service]) for service in services},
      "data": {service: _serialize_service(sm, service) for service in services},
    })
    self.sequence += 1

  def record_output(self, planner):
    self.report.write({
      "type": "planner_output", "sequence": self.sequence - 1, "tick_time_ns": time.monotonic_ns(),
      "a_target_mps2": float(planner.output_a_target),
      "mpc_source": str(planner.mpc.source),
      "traffic_follow": dict(planner.traffic_follow.diagnostic),
      "cut_in": dict(planner.cut_in_predictor.diagnostic) if hasattr(planner, "cut_in_predictor") else None,
      "radar_cut_in": dict(planner.radar_cut_in_predictor.diagnostic) if hasattr(planner, "radar_cut_in_predictor") else None,
      "cut_in_source": getattr(planner, "cut_in_source", None),
    })

  def close(self):
    self.report.close()


class ReplayMaster:
  def __init__(self, snapshot):
    import openpilot.cereal.messaging as messaging

    self.services = tuple(snapshot["data"])
    self.frame = snapshot["frame"]
    self.tick_time_ns = snapshot.get("tick_time_ns")
    self.updated = snapshot["updated"]
    self.alive = snapshot["alive"]
    self.valid = snapshot["valid"]
    self.freq_ok = snapshot["freq_ok"]
    self.logMonoTime = snapshot["log_mono_time_ns"]
    self._all_checks = snapshot["all_checks"]
    self.data = {}
    for service, value in snapshot["data"].items():
      if service == RADAR_SERVICE:
        if value.get("encoding") != "base64":
          raise ValueError("unsupported radar trace encoding")
        payload = base64.b64decode(value["payload"], validate=True)
        if len(payload) > MAX_PAYLOAD_BYTES:
          raise ValueError("radar trace payload too large")
        self.data[service] = payload
        continue
      message = messaging.new_message(service)
      field = getattr(message, service)
      field.from_dict(value)
      self.data[service] = field

  def __getitem__(self, service):
    return self.data[service]

  def all_checks(self, service_list=None):
    if service_list is None:
      return self._all_checks
    return all(self.alive[service] and self.valid[service] and self.freq_ok[service]
               for service in service_list)


def trace_health(report_dir: Path):
  sample_count = gap_count = 0
  previous_sequence = previous_frame = previous_time = None
  first_time = None
  header_count = 0
  try:
    for record in iter_report_records(Path(report_dir) / "planner"):
      if record.get("type") == "planner_header":
        header_count += 1
        continue
      if record.get("type") != "planner_input":
        continue
      sequence, frame, timestamp = record.get("sequence"), record.get("frame"), record.get("tick_time_ns")
      if (sequence is None or frame is None or timestamp is None or
          sequence != (0 if previous_sequence is None else previous_sequence + 1) or
          (previous_frame is not None and frame <= previous_frame) or
          (previous_time is not None and timestamp <= previous_time)):
        gap_count += 1
      previous_sequence, previous_frame, previous_time = sequence, frame, timestamp
      if first_time is None:
        first_time = timestamp
      sample_count += 1
  except (OSError, ValueError):
    gap_count += 1
  return {"planner_trace_valid": header_count == 1 and sample_count >= 2 and gap_count == 0,
          "planner_trace_sample_count": sample_count, "planner_trace_gap_count": gap_count,
          "planner_trace_first_tick_ns": first_time, "planner_trace_last_tick_ns": previous_time}


def trace_hashes(report_dir: Path):
  directory = Path(report_dir) / "planner"
  return {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
          for path in sorted(directory.glob("events*.jsonl"))}


def parity_report_valid(report_dir: Path):
  try:
    result = json.loads((Path(report_dir) / "planner-parity.json").read_text(encoding="utf-8"))
  except (OSError, ValueError):
    return False
  health = trace_health(report_dir)
  return (result.get("mode_zero_parity_verified") is True and
          result.get("mpc_implementation_compared") is True and
          health["planner_trace_valid"] and
          result.get("ticks_compared") == health["planner_trace_sample_count"] and
          result.get("trace_hashes") == trace_hashes(report_dir))


def replay_mode_zero(report_dir: Path, stock_ref: str):
  """Run both planner versions over identical captured SubMaster snapshots."""
  import numpy as np
  from opendbc.car.structs import car
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

  health = trace_health(report_dir)
  result = {"mode_zero_parity_verified": False, "mpc_implementation_compared": False,
            "ticks_compared": 0,
            "stock_ref": stock_ref, "trace_hashes": trace_hashes(report_dir)}
  if not health["planner_trace_valid"]:
    result["reason"] = "planner trace incomplete"
    return result

  root = Path(__file__).resolve().parents[3]
  source_path = "openpilot/selfdrive/controls/lib/longitudinal_planner.py"
  mpc_path = "openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
  mpc_import = "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc"
  old_mpc_source = subprocess.run(["git", "show", f"{stock_ref}:{mpc_path}"], cwd=root,
                                  text=True, capture_output=True, check=True).stdout
  old_mpc = ModuleType("phase1_stock_long_mpc")
  old_mpc.__file__ = mpc_path
  exec(compile(old_mpc_source, mpc_path, "exec"), old_mpc.__dict__)
  result["stock_mpc_sha256"] = hashlib.sha256(old_mpc_source.encode()).hexdigest()
  result["candidate_mpc_sha256"] = hashlib.sha256((root / mpc_path).read_bytes()).hexdigest()
  source = subprocess.run(["git", "show", f"{stock_ref}:{source_path}"], cwd=root,
                          text=True, capture_output=True, check=True).stdout
  stock_module = ModuleType("phase1_stock_longitudinal_planner")
  stock_module.__file__ = source_path
  with patch.dict(sys.modules, {mpc_import: old_mpc}):
    exec(compile(source, source_path, "exec"), stock_module.__dict__)
  result["mpc_implementation_compared"] = True

  records = iter_report_records(Path(report_dir) / "planner")
  header = next(records)
  if header.get("type") != "planner_header":
    result["reason"] = "missing planner header"
    return result
  cp = car.CarParams.new_message()
  cp.from_dict(header["car_params"])
  with patch.dict(os.environ, {"SIMULATION": "1", "VN_TRAFFIC_MODE": "0"}):
    stock = stock_module.LongitudinalPlanner(cp)
    candidate = LongitudinalPlanner(cp)
  for record in records:
    if record.get("type") != "planner_input":
      continue
    sm = ReplayMaster(record)
    stock.update(sm)
    candidate.update(sm)
    result["ticks_compared"] += 1
    if (not np.isclose(stock.output_a_target, candidate.output_a_target, atol=1e-6) or
        stock.output_should_stop != candidate.output_should_stop or
        stock.mpc.source != candidate.mpc.source or
        stock.fcw != candidate.fcw or
        stock.allow_throttle != candidate.allow_throttle or
        not np.allclose(stock.mpc.params, candidate.mpc.params, atol=1e-6) or
        not np.allclose(stock.mpc.x_sol, candidate.mpc.x_sol, atol=1e-6) or
        not np.allclose(stock.mpc.u_sol, candidate.mpc.u_sol, atol=1e-6) or
        not np.allclose(stock.j_desired_trajectory, candidate.j_desired_trajectory, atol=1e-6) or
        not np.allclose(stock.a_desired_trajectory, candidate.a_desired_trajectory, atol=1e-6) or
        not np.allclose(stock.v_desired_trajectory, candidate.v_desired_trajectory, atol=1e-6)):
      result["reason"] = f"planner output differs at sequence {record['sequence']}"
      return result
  result["mode_zero_parity_verified"] = result["ticks_compared"] == health["planner_trace_sample_count"]
  return result


def main():
  parser = argparse.ArgumentParser(description="Verify Phase 2 mode 0 against Phase 1 planner on recorded inputs")
  parser.add_argument("report_dir", type=Path)
  parser.add_argument("--stock-ref", default="6a125380ac8e3b261ed1feb60a9eb8d6d84e18a5")
  args = parser.parse_args()
  result = replay_mode_zero(args.report_dir, args.stock_ref)
  (args.report_dir / "planner-parity.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))
  if not result["mode_zero_parity_verified"]:
    raise SystemExit(1)


if __name__ == "__main__":
  main()
