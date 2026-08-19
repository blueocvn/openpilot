# distutils: language = c++
# cython: boundscheck=False, wraparound=False

import numpy as np
cimport numpy as cnp


cdef extern from "libyuv/convert_from_argb.h" namespace "libyuv":
  int ARGBToNV12(const unsigned char* src_argb, int src_stride_argb,
                 unsigned char* dst_y, int dst_stride_y,
                 unsigned char* dst_uv, int dst_stride_uv,
                 int width, int height)


def bgra_to_nv12(cnp.ndarray[cnp.uint8_t, ndim=3, mode="c"] bgra,
                 int stride, int y_height, int uv_height, int size):
  """Convert CARLA's BGRA image directly to padded NV12 using libyuv."""
  cdef int height = bgra.shape[0]
  cdef int width = bgra.shape[1]
  cdef cnp.ndarray[cnp.uint8_t, ndim=1, mode="c"] output
  cdef int result
  cdef int uv_offset = stride * y_height
  cdef int row
  cdef int column
  cdef int b
  cdef int g
  cdef int r

  if bgra.shape[2] != 4:
    raise ValueError("expected BGRA image with four channels")

  # The padding is part of the VisionIPC buffer contract. Initializing it also
  # matches the former NumPy implementation exactly.
  output = np.zeros(size, dtype=np.uint8)
  result = ARGBToNV12(&bgra[0, 0, 0], width * 4,
                      &output[0], stride,
                      &output[uv_offset], stride,
                      width, height)
  if result:
    raise RuntimeError(f"libyuv ARGBToNV12 failed: {result}")

  # libyuv's luma coefficients have a slightly different rounding convention
  # from openpilot's historical camera kernel. Keep its SIMD chroma conversion
  # but reproduce that luma plane exactly.
  for row in range(height):
    for column in range(width):
      b = bgra[row, column, 0]
      g = bgra[row, column, 1]
      r = bgra[row, column, 2]
      output[row * stride + column] = ((b * 13 + g * 65 + r * 33 + 64) >> 7) + 16

  # The simulator's original OpenCL-equivalent kernel intentionally uses
  # half-amplitude BT.601 chroma. libyuv's U/V coefficients are exactly twice
  # those values, so this maps its result back without changing any output
  # bytes. Do only the valid image extent; VisionIPC padding stays zero.
  for row in range(height // 2):
    for column in range(width):
      output[uv_offset + row * stride + column] = ((<int>output[uv_offset + row * stride + column] - 127) >> 1) + 128
  return output.tobytes()
