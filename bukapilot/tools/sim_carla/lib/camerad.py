"""Simulated camerad for bukapilot.

bukapilot (openpilot 0.8.13) differs from current openpilot here:
  * modeld reads planar I420 from VISION_STREAM_ROAD (current openpilot uses
    padded NV12 from VISION_STREAM_NARROW_ROAD).
  * the UI draws VISION_STREAM_RGB_BACK, a packed BGR buffer.
  * the frame metadata goes out on 'roadCameraState'.

The legacy tools/sim bridge converts with an OpenCL kernel and therefore
needs pyopencl plus a working OpenCL ICD. bgra_to_i420() below reproduces
selfdrive/camerad/transforms/rgb_to_yuv.cl bit-for-bit in NumPy, so this bridge
runs without OpenCL (tests/test_sim_carla.py checks it against the kernel).
"""
import numpy as np

import cereal.messaging as messaging
from cereal.visionipc.visionipc_pyx import VisionIpcServer, VisionStreamType  # pylint: disable=no-name-in-module, import-error
from common.realtime import sec_since_boot
from tools.sim_carla.lib.common import W, H


def bgra_to_i420(bgra):
  """BGRA (H, W, 4) uint8 -> planar I420 bytes, identical to rgb_to_yuv.cl."""
  h, w = bgra.shape[:2]
  b = bgra[:, :, 0].astype(np.int32)
  g = bgra[:, :, 1].astype(np.int32)
  r = bgra[:, :, 2].astype(np.int32)

  y = (((b * 13 + g * 65 + r * 33) + 64) >> 7) + 16

  # The kernel's AVERAGE() is (sum of the 2x2 block + 1) >> 1, i.e. twice the
  # mean. Together with the halved coefficients and >> 8 that is BT.601.
  def avg2(c):
    return (c[0::2, 0::2] + c[0::2, 1::2] + c[1::2, 0::2] + c[1::2, 1::2] + 1) >> 1

  ab, ag, ar = avg2(b), avg2(g), avg2(r)
  u = (ab * 56 - ag * 37 - ar * 19 + 0x8080) >> 8
  v = (ar * 56 - ag * 47 - ab * 9 + 0x8080) >> 8

  out = np.empty(w * h * 3 // 2, dtype=np.uint8)
  out[:w * h] = y.astype(np.uint8).ravel()
  out[w * h:w * h * 5 // 4] = u.astype(np.uint8).ravel()
  out[w * h * 5 // 4:] = v.astype(np.uint8).ravel()
  return out


class Camerad:
  """Simulates the camerad daemon (road camera only; 0.8.13 modeld is single camera)."""

  def __init__(self):
    self.pm = messaging.PubMaster(['roadCameraState'])
    self.frame_id = 0

    self.vipc_server = VisionIpcServer("camerad")
    # TODO (same as legacy bridge): drop RGB once the UI reads YUV.
    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_RGB_BACK, 4, True, W, H)
    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 40, False, W, H)
    self.vipc_server.start_listener()

  def send_bgra(self, bgra):
    assert bgra.shape == (H, W, 4), f"{bgra.shape}"
    yuv = bgra_to_i420(bgra)
    bgr = np.ascontiguousarray(bgra[:, :, :3])

    # Real monotonic timestamps: the legacy bridge sends frame_id * 0.05, which
    # stays near zero and makes every consumer's frame-age check meaningless.
    eof = int(sec_since_boot() * 1e9)
    self.vipc_server.send(VisionStreamType.VISION_STREAM_RGB_BACK, bgr.tobytes(), self.frame_id, eof, eof)
    self.vipc_server.send(VisionStreamType.VISION_STREAM_ROAD, yuv.tobytes(), self.frame_id, eof, eof)

    dat = messaging.new_message('roadCameraState')
    dat.roadCameraState = {
      "frameId": self.frame_id,
      "timestampEof": eof,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0],
    }
    self.pm.send('roadCameraState', dat)
    self.frame_id += 1
