import time

import numpy as np

from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcServer
from openpilot.cereal import messaging

from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.tools.sim.lib.common import W, H

try:
  from openpilot.tools.sim.lib.libyuv_pyx import bgra_to_nv12 as _libyuv_bgra_to_nv12
except ImportError:
  # A source checkout can run the simulator before its optional SCons targets
  # have been built. Keep the portable NumPy path available in that case.
  _libyuv_bgra_to_nv12 = None


def rgb_to_nv12(rgb):
  """Convert RGB image to NV12 (YUV420) format using BT.601 coefficients."""
  h, w = rgb.shape[:2]
  # All intermediate values fit in int16: the largest luma numerator is
  # 255 * (13 + 65 + 33) + 64 = 28369.  int32 triples the bandwidth and
  # temporary allocation size for these 1928x1208 simulator frames.
  r = rgb[:, :, 0].astype(np.int16)
  g = rgb[:, :, 1].astype(np.int16)
  b = rgb[:, :, 2].astype(np.int16)

  # Y plane - BT.601 coefficients (matches original OpenCL kernel)
  y = ((((b * 13 + g * 65 + r * 33) + 64) >> 7) + 16).astype(np.uint8)

  # Subsample RGB for UV (2x2 box filter)
  r_sub = (r[0::2, 0::2] + r[0::2, 1::2] + r[1::2, 0::2] + r[1::2, 1::2] + 2) >> 2
  g_sub = (g[0::2, 0::2] + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2] + 2) >> 2
  b_sub = (b[0::2, 0::2] + b[0::2, 1::2] + b[1::2, 0::2] + b[1::2, 1::2] + 2) >> 2

  # U and V planes
  # Split 0x8080 into the rounding offset and chroma center.  This avoids
  # overflowing signed int16 while keeping the result bit-identical.
  u = (((b_sub * 56 - g_sub * 37 - r_sub * 19 + 128) >> 8) + 128).astype(np.uint8)
  v = (((r_sub * 56 - g_sub * 47 - b_sub * 9 + 128) >> 8) + 128).astype(np.uint8)

  # Interleave UV for NV12 format
  uv = np.empty((h // 2, w), dtype=np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v

  # modeld's compiled warp uses the same padded NV12 layout as device camerad.
  # VisionIPC's convenience create_buffers() uses a tightly packed layout,
  # which is too small and has the wrong UV offset for the compiled model.
  stride, y_height, uv_height, size = get_nv12_info(w, h)
  frame = np.zeros(size, dtype=np.uint8)
  frame[:stride * y_height].reshape(y_height, stride)[:h, :w] = y
  uv_offset = stride * y_height
  frame[uv_offset:uv_offset + stride * uv_height].reshape(uv_height, stride)[:h // 2, :w] = uv
  return frame.tobytes()


class Camerad:
  """Simulates the camerad daemon"""
  def __init__(self, dual_camera):
    self.pm = messaging.PubMaster(['narrowRoadCameraState', 'wideRoadCameraState'])

    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    stride, y_height, _, size = get_nv12_info(W, H)
    uv_offset = stride * y_height
    self.vipc_server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_NARROW_ROAD, 5, W, H, size, stride, uv_offset)
    if dual_camera:
      self.vipc_server.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H, size, stride, uv_offset)

    self.vipc_server.start_listener()

  def cam_send_yuv_road(self, yuv, timestamp_ns=None):
    self._send_yuv(yuv, self.frame_road_id, 'narrowRoadCameraState', VisionStreamType.VISION_STREAM_NARROW_ROAD, timestamp_ns)
    self.frame_road_id += 1

  def cam_send_yuv_wide_road(self, yuv, timestamp_ns=None):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD, timestamp_ns)
    self.frame_wide_id += 1

  def rgb_to_yuv(self, rgb):
    """Convert RGB to NV12 YUV format."""
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8
    return rgb_to_nv12(rgb)

  def bgra_to_yuv(self, bgra):
    """Convert a CARLA BGRA frame directly to padded NV12."""
    assert bgra.shape == (H, W, 4), f"{bgra.shape}"
    assert bgra.dtype == np.uint8
    if _libyuv_bgra_to_nv12 is not None:
      stride, y_height, uv_height, size = get_nv12_info(W, H)
      return _libyuv_bgra_to_nv12(bgra, stride, y_height, uv_height, size)
    return rgb_to_nv12(np.ascontiguousarray(bgra[:, :, 2::-1]))

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type, timestamp_ns=None):
    # Consumers compare camera timestamps against the system monotonic clock.
    # A frame-relative timestamp starting at zero is rejected as stale.
    eof = timestamp_ns if timestamp_ns is not None else time.monotonic_ns()
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)
