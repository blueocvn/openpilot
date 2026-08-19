import os
import subprocess
import time
from multiprocessing import Queue

import numpy as np
import pytest

from openpilot.cereal import messaging

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.tools.sim.bridge.carla.carla_bridge import CarlaBridge
from openpilot.tools.sim.bridge.carla.carla_world import CarlaWorld
from openpilot.tools.sim.bridge.common import control_cmd_gen


class FakeImage:
  width = 2
  height = 1
  raw_data = bytes((0, 0, 255, 255, 0, 255, 0, 255))


def test_carla_bgra_to_rgb():
  rgb = CarlaWorld._rgb(FakeImage())
  np.testing.assert_array_equal(rgb, [[[255, 0, 0], [0, 255, 0]]])
  assert rgb.flags.c_contiguous


@pytest.mark.slow
@pytest.mark.skipif(os.environ.get("CARLA_INTEGRATION") != "1", reason="requires a running CARLA server")
@pytest.mark.parametrize("openpilot_longitudinal", [False, True])
def test_carla_closed_loop(openpilot_longitudinal):
  params = Params()
  params.remove("Offroad_ExcessiveActuation")
  params.put_bool("AlphaLongitudinalEnabled", openpilot_longitudinal, block=True)
  sim_dir = os.path.join(BASEDIR, "openpilot", "tools", "sim")
  manager_env = os.environ.copy()
  manager_env["CI"] = "1"
  manager_env["FINGERPRINT"] = "TESLA_MODEL_3"
  manager_env["SIMULATOR"] = "carla"
  manager_env["BLOCK"] = f"{manager_env.get('BLOCK', '')},soundd"
  manager = subprocess.Popen(["./launch_openpilot.sh"], cwd=sim_dir, env=manager_env)
  commands = Queue()
  bridge = CarlaBridge(dual_camera=False, high_quality=False, openpilot_longitudinal=openpilot_longitudinal)
  bridge_process = bridge.run(commands)
  sm = messaging.SubMaster(["selfdriveState", "carState"])

  try:
    deadline = time.monotonic() + 180
    engaged = False
    moving = False
    while time.monotonic() < deadline:
      sm.update(1000)
      engaged |= sm["selfdriveState"].active
      moving |= sm["carState"].vEgo > 1.0
      if engaged and moving:
        break
      assert bridge_process.is_alive(), f"bridge exited with code {bridge_process.exitcode}"

    assert bridge.started.value, "CARLA bridge did not start"
    assert engaged, "openpilot did not engage"
    assert moving, "CARLA ego vehicle did not move"
  finally:
    commands.put(control_cmd_gen("quit"))
    bridge_process.join(15)
    if bridge_process.is_alive():
      bridge_process.terminate()
      bridge_process.join(10)
    manager.terminate()
    try:
      manager.wait(15)
    except subprocess.TimeoutExpired:
      manager.kill()
