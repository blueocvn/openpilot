"""Simulation-only receipt for the compiled model artifact modeld actually loaded."""

import hashlib
import json
import time
from pathlib import Path

from openpilot.common.file_chunker import get_existing_chunks


def record_loaded_artifact(report_dir: Path, artifact_base: Path):
  artifact_base = Path(artifact_base).resolve()
  paths = [Path(path) for path in get_existing_chunks(artifact_base)]
  receipt = {"artifact_name": artifact_base.name, "artifact_path": str(artifact_base),
             "loaded_at_monotonic_ns": time.monotonic_ns(),
             "artifact_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}}
  report_dir = Path(report_dir)
  report_dir.mkdir(parents=True, exist_ok=True)
  (report_dir / "model-runtime.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
  return receipt


def verify_runtime_artifact(manifest, receipt):
  recorded = receipt.get("artifact_sha256") or {}
  expected = manifest.get("compiled_artifact_sha256") or {}
  artifact_name = receipt.get("artifact_name")
  if artifact_name not in ("driving_tinygrad.pkl", "big_driving_tinygrad.pkl"):
    return False
  selected = {name: digest for name, digest in expected.items()
              if name == artifact_name or name.startswith(f"{artifact_name}.chunk")}
  return bool(selected) and recorded == selected


def verify_compiled_source_link(manifest, build_receipt, runtime_receipt):
  """Verify the source link from hashed ONNX through a build receipt to runtime.

  A manifest flag is deliberately ignored: it is descriptive, not evidence.
  """
  if not isinstance(build_receipt, dict):
    return False, "missing build receipt"
  onnx_name = build_receipt.get("onnx_name")
  expected_onnx = (manifest.get("onnx_sha256") or {}).get(onnx_name)
  if not onnx_name or not expected_onnx or build_receipt.get("onnx_sha256") != expected_onnx:
    return False, "build receipt ONNX does not match manifest"
  if not build_receipt.get("compiler_revision") or not build_receipt.get("compiler_args"):
    return False, "build receipt lacks compiler identity or arguments"
  artifact_name = build_receipt.get("artifact_name")
  build_time, runtime_time = build_receipt.get("created_at_monotonic_ns"), runtime_receipt.get("loaded_at_monotonic_ns")
  if not isinstance(build_time, int) or not isinstance(runtime_time, int) or build_time > runtime_time:
    return False, "build or runtime receipt is stale or unassociated"
  build_hashes = build_receipt.get("artifact_sha256") or {}
  runtime_hashes = runtime_receipt.get("artifact_sha256") or {}
  if artifact_name != runtime_receipt.get("artifact_name"):
    return False, "runtime artifact differs from build output"
  expected_artifact = {name: digest for name, digest in (manifest.get("compiled_artifact_sha256") or {}).items()
                       if name == artifact_name or name.startswith(f"{artifact_name}.chunk")}
  if not expected_artifact or build_hashes != expected_artifact:
    return False, "build output hashes do not match manifest"
  if runtime_hashes != build_hashes:
    return False, "runtime hashes do not match build output"
  return True, None
