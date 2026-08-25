#!/usr/bin/env python3
import argparse
import os
import subprocess

from typing import Any
from multiprocessing import Queue

def create_bridge(dual_camera, high_quality, simulator="metadrive", **kwargs):
  queue: Any = Queue()

  if simulator == "carla":
    from openpilot.tools.sim.bridge.carla.carla_bridge import CarlaBridge
    simulator_bridge = CarlaBridge(dual_camera, high_quality, **kwargs)
  else:
    from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge
    simulator_bridge = MetaDriveBridge(dual_camera, high_quality, **kwargs)
  simulator_process = simulator_bridge.run(queue)

  return queue, simulator_process, simulator_bridge

def main():
  _, simulator_process, _ = create_bridge(True, False)
  simulator_process.join()

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--openpilot-longitudinal', action='store_true',
                      help='let openpilot control CARLA throttle and braking instead of stock ACC')
  parser.add_argument('--simulator', choices=('metadrive', 'carla'), default='metadrive')
  parser.add_argument('--carla-host', default=None, help='CARLA server host (default: CARLA_HOST or WSL gateway)')
  parser.add_argument('--carla-port', type=int, default=2000)
  # Town10HD_Opt spawn 16 (the old default) has only ~155-175m of road before
  # a junction -- guaranteed to hit one within seconds at any real speed, with
  # no navigation to route around it. Town04_Opt spawn 40 has an 855m
  # junction-free highway stretch, measured by walking the lane graph forward
  # from every spawn point in both maps.
  parser.add_argument('--carla-town', default='Town04_Opt')
  parser.add_argument('--carla-spawn-point', type=int, default=40)
  parser.add_argument('--launch-openpilot', action='store_true', help='start and own the simulator manager process')

  return parser.parse_args(add_args)

if __name__ == "__main__":
  args = parse_args()

  manager = None
  manager_log = None
  if args.launch_openpilot:
    from openpilot.common.basedir import BASEDIR
    from openpilot.common.params import Params

    # This safety alert can remain after an interrupted simulator run and blocks
    # all onroad processes on the next launch. It is simulator state, not a real
    # vehicle fault, so clear only this alert before starting manager.
    params = Params()
    params.remove("Offroad_ExcessiveActuation")
    # Set this before manager/card starts to avoid a fingerprinting race with
    # bridge construction. Leave MetaDrive's existing setup untouched.
    is_metadrive = args.simulator == "metadrive"
    if not is_metadrive:
      params.put_bool("AlphaLongitudinalEnabled", args.openpilot_longitudinal, block=True)

    sim_dir = os.path.join(BASEDIR, "openpilot", "tools", "sim")
    state_dir = ".metadrive" if is_metadrive else ".carla"
    manager_log_path = os.environ.get("OPENPILOT_MANAGER_LOG", os.path.join(BASEDIR, state_dir, "logs", "openpilot.log"))
    os.makedirs(os.path.dirname(manager_log_path), exist_ok=True)
    manager_log = open(manager_log_path, "w")
    manager_env = os.environ.copy()
    manager = subprocess.Popen(["./launch_openpilot.sh"], cwd=sim_dir, env=manager_env,
                               stdout=manager_log, stderr=subprocess.STDOUT)

  bridge_args = {}
  if args.simulator == 'carla':
    bridge_args = {
      'host': args.carla_host,
      'port': args.carla_port,
      'town': args.carla_town,
      'spawn_point': args.carla_spawn_point,
      'openpilot_longitudinal': args.openpilot_longitudinal,
    }
  try:
    queue, simulator_process, simulator_bridge = create_bridge(
      args.dual_camera, args.high_quality, args.simulator, **bridge_args,
    )

    if args.joystick:
      # start input poll for joystick
      from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread

      wheel_poll_thread(queue)
    else:
      # start input poll for keyboard
      from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

      keyboard_poll_thread(queue)
  finally:
    if 'simulator_bridge' in locals():
      simulator_bridge.shutdown()
    if 'simulator_process' in locals():
      simulator_process.join(15)
      if simulator_process.is_alive():
        simulator_process.terminate()
        simulator_process.join(10)
    if manager is not None:
      manager.terminate()
      try:
        manager.wait(15)
      except subprocess.TimeoutExpired:
        manager.kill()
    if manager_log is not None:
      manager_log.close()
