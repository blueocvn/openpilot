import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from openpilot.tools.sim.model_provenance import record_loaded_artifact, verify_compiled_source_link, verify_runtime_artifact


class TestModelProvenance(unittest.TestCase):
  def test_loaded_model_receipt_matches_manifest_hashes(self):
    with TemporaryDirectory() as directory:
      root = Path(directory)
      artifact = root / "driving_tinygrad.pkl"
      artifact.write_bytes(b"compiled-model")
      receipt = record_loaded_artifact(root / "report", artifact)
      manifest = {"compiled_artifact_sha256": {
        artifact.name: hashlib.sha256(b"compiled-model").hexdigest()}}
      self.assertTrue(verify_runtime_artifact(manifest, receipt))
      self.assertEqual(json.loads((root / "report" / "model-runtime.json").read_text())["artifact_name"], artifact.name)
      artifact.write_bytes(b"different")
      self.assertFalse(verify_runtime_artifact(manifest, record_loaded_artifact(root / "other", artifact)))

  def test_receipt_must_cover_every_chunk_of_selected_artifact(self):
    manifest = {"compiled_artifact_sha256": {
      "driving_tinygrad.pkl.chunk0": "zero",
      "driving_tinygrad.pkl.chunk1": "one",
      "big_driving_tinygrad.pkl.chunk0": "other",
    }}
    receipt = {"artifact_name": "driving_tinygrad.pkl",
               "artifact_sha256": {"driving_tinygrad.pkl.chunk0": "zero"}}
    self.assertFalse(verify_runtime_artifact(manifest, receipt))
    receipt["artifact_sha256"]["driving_tinygrad.pkl.chunk1"] = "one"
    self.assertTrue(verify_runtime_artifact(manifest, receipt))

  def test_compiled_source_link_is_derived_from_receipts(self):
    manifest = {"onnx_sha256": {"driving_supercombo.onnx": "onnx"},
                "compiled_artifact_sha256": {"driving_tinygrad.pkl": "chunk"},
                "compiled_source_link_verified": True}
    build = {"onnx_name": "driving_supercombo.onnx", "onnx_sha256": "onnx",
             "compiler_revision": "abc", "compiler_args": ["--onnx", "driving_supercombo.onnx"],
             "artifact_name": "driving_tinygrad.pkl", "artifact_sha256": {"driving_tinygrad.pkl": "chunk"},
             "created_at_monotonic_ns": 100}
    runtime = {"artifact_name": "driving_tinygrad.pkl", "artifact_sha256": {"driving_tinygrad.pkl": "chunk"},
               "loaded_at_monotonic_ns": 200}
    self.assertTrue(verify_compiled_source_link(manifest, build, runtime)[0])
    self.assertFalse(verify_compiled_source_link(manifest, {}, runtime)[0])
    self.assertFalse(verify_compiled_source_link({**manifest, "onnx_sha256": {"driving_supercombo.onnx": "other"}}, build, runtime)[0])
    self.assertFalse(verify_compiled_source_link(manifest, {**build, "compiler_args": []}, runtime)[0])
    self.assertFalse(verify_compiled_source_link(manifest, build, {**runtime, "artifact_sha256": {}})[0])
    self.assertFalse(verify_compiled_source_link(manifest, {**build, "created_at_monotonic_ns": 300}, runtime)[0])


if __name__ == "__main__":
  unittest.main()
