import json
import os
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from openpilot.tools.sim.bridge.carla.scenes.motorcycle_weave import MotorcycleWeaveScene


class TestTrafficManifest(unittest.TestCase):
  def test_manifest_records_opt_in_and_profile(self):
    with TemporaryDirectory() as directory:
      scene = MotorcycleWeaveScene(object(), object(), object(), [], report_dir=directory)
      with patch.dict(os.environ, {"VN_TRAFFIC_MODE": "1", "VN_TRAFFIC_PROFILE": "gentle"}):
        with patch("openpilot.tools.sim.bridge.carla.scenes.motorcycle_weave.sha256_path", return_value="hash"):
          scene._write_manifest()
      manifest = json.loads((scene.report_dir / "manifest.json").read_text())
      self.assertEqual(manifest["vn_traffic_mode"], "1")
      self.assertEqual(manifest["vn_traffic_profile"], "gentle")
      self.assertIn("openpilot/selfdrive/controls/lib/vn_traffic_policy.py", manifest["source_sha256"])


if __name__ == "__main__":
  unittest.main()
