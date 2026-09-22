import os
import subprocess
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pyray as rl

from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.selfdrive.ui.onroad import augmented_road_view
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView, DEFAULT_DEVICE_CAMERA, NARROW_ROAD_CAM


CARLA_RUN_SH = Path(__file__).resolve().parents[1] / "carla" / "run.sh"


class TestCarlaUi(unittest.TestCase):
  def test_monitor_view_contains_the_whole_camera_and_keeps_model_overlay_aligned(self):
    """Catches enlarged CARLA UI clipping the road-camera frame or misaligning model drawings."""
    rect = rl.Rectangle(30, 30, 2100, 1020)
    view = AugmentedRoadView.__new__(AugmentedRoadView)
    view._content_rect = rect
    view._stream_type = NARROW_ROAD_CAM
    view.device_camera = DEFAULT_DEVICE_CAMERA
    view.view_from_calib = view_frame_from_device_frame.copy()
    view.view_from_wide_calib = view_frame_from_device_frame.copy()
    view._matrix_cache_key = None
    view._cached_matrix = None
    view.model_renderer = Mock()
    fake_ui_state = SimpleNamespace(sm=SimpleNamespace(recv_frame={"extrinsicsCalibration": 0}))

    with patch.object(augmented_road_view, "ui_state", fake_ui_state), \
         patch.dict(os.environ, {"CARLA_MONITOR_FULL_FRAME": "1"}):
      matrix = view._calc_frame_matrix(rect)

    # 1928x1208 camera fits inside a 2100x1020 viewport at 1020/1208 scale.
    self.assertAlmostEqual(rect.width * matrix[0, 0], 1928 * 1020 / 1208, places=3)
    self.assertAlmostEqual(rect.height * matrix[1, 1], 1020.0, places=3)
    self.assertAlmostEqual(matrix[0, 2], 0.0)
    self.assertAlmostEqual(matrix[1, 2], 0.0)
    model_transform = view.model_renderer.set_transform.call_args.args[0]
    camera_projection = DEFAULT_DEVICE_CAMERA.narrow_road.intrinsics @ view.view_from_calib
    video_transform = model_transform @ np.linalg.inv(camera_projection)
    self.assertAlmostEqual(video_transform[0, 0], 1020 / 1208, places=3)
    self.assertAlmostEqual(video_transform[1, 1], 1020 / 1208, places=3)

  def test_launcher_defaults_to_big_ui_and_auto_scale_without_overriding_explicit_choices(self):
    """Catches the CARLA launcher reverting to the tiny 536x240 UI or forcing 3x scaling."""
    script = 'source "$1" status >/dev/null; printf "%s|%s|%s" "${BIG-unset}" "${SCALE-unset}" "${CARLA_MONITOR_FULL_FRAME-unset}"'
    env = os.environ.copy()
    for name in ("BIG", "SCALE", "CARLA_MONITOR_FULL_FRAME"):
      env.pop(name, None)
    default = subprocess.run(["bash", "-c", script, "bash", str(CARLA_RUN_SH)],
                             env=env, capture_output=True, text=True, check=True)
    self.assertEqual(default.stdout, "1|unset|1")

    env.update({"BIG": "0", "SCALE": "2.0", "CARLA_MONITOR_FULL_FRAME": "0"})
    override = subprocess.run(["bash", "-c", script, "bash", str(CARLA_RUN_SH)],
                              env=env, capture_output=True, text=True, check=True)
    self.assertEqual(override.stdout, "0|2.0|0")
