#!/usr/bin/env python3
"""Bridge between CARLA and bukapilot (upstream openpilot tools/sim layout).

Normally started inside the bukapilot-sim:local container by start_docker.sh.
"""
import argparse
import os
import signal
import subprocess
import sys
from multiprocessing import Queue

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
BASEDIR = os.path.abspath(os.path.join(SIM_DIR, "../.."))
if BASEDIR not in sys.path:
  sys.path.insert(0, BASEDIR)


def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between CARLA and bukapilot.')
  parser.add_argument('--joystick', action='store_true', help='read /dev/input/js0 instead of the keyboard')
  parser.add_argument('--low_quality', action='store_true', help='unload foliage, buildings, props, ... from the map')
  parser.add_argument('--town', default='Town04_Opt')
  parser.add_argument('--spawn_point', type=int, default=16)
  parser.add_argument('--vehicle', default='vehicle.tesla.model3', help='CARLA blueprint for the ego car')
  parser.add_argument('--weather', default='ClearNoon', help='carla.WeatherParameters preset')
  parser.add_argument('--host', default=os.environ.get('CARLA_HOST', '127.0.0.1'))
  parser.add_argument('--port', type=int, default=int(os.environ.get('CARLA_PORT', '2000')))
  parser.add_argument('--no-auto-engage', dest='auto_engage', action='store_false',
                      help='do not press SET automatically; engage with key 2 (or 1)')
  parser.add_argument('--launch-openpilot', action='store_true',
                      help='start bukapilot (launch_openpilot.sh) and own its lifetime')
  parser.add_argument('--duration', type=float, default=None,
                      help='headless run: no keyboard, quit after this many seconds')
  return parser.parse_args(add_args)


def main():
  args = parse_args()

  from tools.sim_carla.bridge.carla_bridge import CarlaBridge
  from tools.sim_carla.lib.params import set_params_enabled

  manager = manager_log = None
  if args.launch_openpilot:
    # set params before manager starts so controlsd never sees a half-set state
    set_params_enabled()
    log_path = os.environ.get("OPENPILOT_MANAGER_LOG", os.path.join(BASEDIR, ".carla", "logs", "bukapilot.log"))
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    manager_log = open(log_path, "w")
    print(f"bukapilot manager log: {log_path}", flush=True)
    manager = subprocess.Popen(["./launch_openpilot.sh"], cwd=SIM_DIR, env=os.environ.copy(),
                               stdout=manager_log, stderr=subprocess.STDOUT, start_new_session=True)

  relay = None
  if os.environ.get("OP_UI", "0") == "1":
    # stream bukapilot state + camera to openpilot's UI on the host (op_ui/host_ui.py)
    relay = subprocess.Popen([sys.executable, os.path.join(SIM_DIR, "lib", "ui_relay.py")],
                             env=os.environ.copy(), start_new_session=True)

  queue = Queue()
  bridge = CarlaBridge(host=args.host, port=args.port, town=args.town, spawn_point=args.spawn_point,
                       vehicle=args.vehicle, low_quality=args.low_quality, weather=args.weather,
                       auto_engage=args.auto_engage, test_run=args.duration is not None)
  bridge_p = bridge.run(queue)
  try:
    if args.duration is not None:
      from tools.sim_carla.lib.keyboard_ctrl import timed_poll_thread
      timed_poll_thread(queue, args.duration, bridge_p)
    elif args.joystick:
      from tools.sim_carla.lib.manual_ctrl import wheel_poll_thread
      wheel_poll_thread(queue)
    else:
      from tools.sim_carla.lib.keyboard_ctrl import keyboard_poll_thread
      keyboard_poll_thread(queue)
    bridge_p.join(30)
  finally:
    if bridge_p.is_alive():
      bridge_p.terminate()
      bridge_p.join(10)
    if relay is not None:
      relay.terminate()
    if manager is not None:
      # manager.py stops its managed processes on KeyboardInterrupt
      manager.send_signal(signal.SIGINT)
      try:
        manager.wait(20)
      except subprocess.TimeoutExpired:
        pass
      try:
        os.killpg(manager.pid, signal.SIGKILL)  # anything left in the session
      except ProcessLookupError:
        pass
    if manager_log is not None:
      manager_log.close()
  return bridge_p.exitcode or 0


if __name__ == "__main__":
  sys.exit(main())
