#!/usr/bin/env python3
"""Offline tests for tools/sim (no CARLA server needed).

  PYTHONPATH=<bukapilot root> python3 -m unittest discover -s tools/sim/tests -t . -v
"""
import json
import os
import unittest

import numpy as np

import cereal.messaging as messaging
from cereal import car
from common.basedir import BASEDIR
from common.features import Features
from common.params import Params
from selfdrive.car.honda.carcontroller import CarController
from selfdrive.car.honda.carstate import CarState
from selfdrive.car.honda.interface import CarInterface
from selfdrive.car.honda.values import CAR, CruiseButtons
from tools.sim.lib.camerad import bgra_to_i420
from tools.sim.lib.common import H, W, SimulatorState, vec3
from tools.sim.lib.simulated_car import SimulatedCar


def opencl_i420(bgra):
  """Reference: the legacy bridge's OpenCL conversion (rgb_to_yuv.cl)."""
  import pyopencl as cl  # pylint: disable=import-error
  import pyopencl.array as cl_array  # pylint: disable=import-error
  ctx = cl.create_some_context(interactive=False)
  q = cl.CommandQueue(ctx)
  h, w = bgra.shape[:2]
  args = f" -DHEIGHT={h} -DWIDTH={w} -DRGB_STRIDE={w*3} -DUV_WIDTH={w // 2} -DUV_HEIGHT={h // 2} -DRGB_SIZE={w * h} "
  src = open(os.path.join(BASEDIR, "selfdrive", "camerad", "transforms", "rgb_to_yuv.cl")).read()
  krnl = cl.Program(ctx, src).build(args).rgb_to_yuv
  rgb = np.ascontiguousarray(bgra[:, :, :3]).reshape((h, w * 3))
  rgb_cl = cl_array.to_device(q, rgb)
  yuv_cl = cl_array.empty_like(rgb_cl)
  krnl(q, (np.int32(w // 4), np.int32(h // 4)), None, rgb_cl.data, yuv_cl.data).wait()
  return np.resize(yuv_cl.get(), np.int32(rgb.size / 2))


class TestCamerad(unittest.TestCase):
  def test_i420_layout(self):
    bgra = np.zeros((H, W, 4), dtype=np.uint8)
    bgra[..., 2] = 255  # pure red
    yuv = bgra_to_i420(bgra)
    self.assertEqual(yuv.size, W * H * 3 // 2)
    y, u, v = yuv[:W * H], yuv[W * H:W * H * 5 // 4], yuv[W * H * 5 // 4:]
    # BT.601 limited range red: Y~82, U~90, V~240
    self.assertTrue(np.all(np.abs(y.astype(int) - 82) <= 1))
    self.assertTrue(np.all(np.abs(u.astype(int) - 90) <= 1))
    self.assertTrue(np.all(np.abs(v.astype(int) - 240) <= 1))

  def test_matches_legacy_opencl_kernel(self):
    try:
      import pyopencl  # noqa: F401  pylint: disable=unused-import,import-error
      os.environ.setdefault("PYOPENCL_CTX", "0")
      bgra = np.random.RandomState(0).randint(0, 256, size=(H, W, 4), dtype=np.uint8)
      ref = opencl_i420(bgra)
    except Exception as e:  # no pyopencl / no OpenCL ICD
      self.skipTest(f"OpenCL unavailable: {e}")
    np.testing.assert_array_equal(bgra_to_i420(bgra), ref)


class TestSimulatedCar(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    # bukapilot's CarInterfaceBase reads FeaturesDict, which manager_init() normally writes.
    # Same feature bits as selfdrive/manager/manager.py, all disabled ("default" package).
    names = ["MyviAzri", "MyviKevin", "ClearCode", "StockAcc", "IgnoreHardIgnition", "IgnoreDM", "LKSTactile"]
    params = Params()
    if params.get("FeaturesDict") is None:
      features = {n: 1 << i for i, n in enumerate(names)}
      params.put("FeaturesDict", json.dumps({"features": features, "packages": {"default": []}, "version": 0}))
      Features().set_package("default")

  def _parse(self, state, frames=20):
    """Feed SimulatedCar CAN through bukapilot's own Honda CarState."""
    sim = SimulatedCar()
    sock = messaging.sub_sock('can', conflate=False, timeout=100)

    fingerprint = {i: {} for i in range(3)}
    fingerprint[0][0x201] = 6  # GAS_SENSOR -> comma pedal
    CP = CarInterface.get_params(CAR.CIVIC, fingerprint)
    CI = CarInterface(CP, CarController, CarState)
    cs = None
    for _ in range(frames):
      sim.update(state)
      cans = messaging.drain_sock_raw(sock, wait_for_one=True)
      cs = CI.update(car.CarControl.new_message(), cans)
    return CP, cs

  def test_carstate(self):
    s = SimulatorState()
    s.valid = True
    s.velocity = vec3(10.0, 0.0, 0.0)
    s.steering_angle = 12.5
    CP, cs = self._parse(s)

    self.assertTrue(CP.enableGasInterceptor)
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.pcmCruise)
    self.assertEqual(cs.gearShifter, car.CarState.GearShifter.drive)
    self.assertAlmostEqual(cs.vEgoRaw, 10.0, delta=0.1)
    self.assertAlmostEqual(cs.steeringAngleDeg, 12.5, delta=0.2)
    self.assertTrue(cs.cruiseState.available)
    self.assertFalse(cs.doorOpen)
    self.assertFalse(cs.seatbeltUnlatched)
    self.assertFalse(cs.gasPressed)
    self.assertFalse(cs.brakePressed)
    self.assertFalse(cs.steeringPressed)
    self.assertTrue(cs.canValid)

  def test_driver_inputs(self):
    s = SimulatorState()
    s.valid = True
    s.user_gas = 0.5
    s.user_brake = 1.0
    s.user_torque = -0.15 * -10000
    s.left_blinker = True
    s.cruise_button = CruiseButtons.DECEL_SET
    _, cs = self._parse(s)
    self.assertTrue(cs.gasPressed)
    self.assertTrue(cs.brakePressed)
    self.assertTrue(cs.steeringPressed)
    self.assertTrue(cs.leftBlinker)


if __name__ == "__main__":
  unittest.main()
