import sys
import termios
import time
from termios import (BRKINT, CS8, CSIZE, ECHO, ICANON, ICRNL, IEXTEN, INPCK,
                     ISTRIP, IXON, PARENB, VMIN, VTIME)

from tools.sim.bridge.common import control_cmd_gen

# Indexes for termios list.
IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6

KEYBOARD_HELP = """
  | key  |   functionality        |
  |------|------------------------|
  |  1   | Cruise resume / accel  |
  |  2   | Cruise set / decel     |
  |  3   | Cruise cancel          |
  |  r   | Reset simulation       |
  |  i   | Toggle ignition        |
  |  h   | Hold / release car     |
  |  z/x | Left / right blinker (latched: press again to turn off) |
  |  q   | Exit all               |
  | wasd | Control manually       |

  These keys also work while the CARLA window has focus (Linux Docker).
  With a blinker on, the car engaged and a lane change possible, a/d in the
  blinker's direction only nudges the steering wheel: openpilot does the lane
  change (assisted lane change). Otherwise a/d steer by hand.
  The blinker stays on until you press z/x again.
"""

KEY_COMMANDS = {
  '1': "cruise_up",
  '2': "cruise_down",
  '3': "cruise_cancel",
  'w': "throttle_1.0",
  'a': "steer_-0.15",
  's': "brake_1.0",
  'd': "steer_0.15",
  'z': "blinker_left",
  'x': "blinker_right",
  'i': "ignition",
  'h': "hold",
  'r': "reset",
  'q': "quit",
}


def getch() -> str:
  stdin_fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(stdin_fd)
  try:
    mode = old_settings.copy()
    mode[IFLAG] &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON)
    mode[CFLAG] &= ~(CSIZE | PARENB)
    mode[CFLAG] |= CS8
    mode[LFLAG] &= ~(ECHO | ICANON | IEXTEN)
    mode[CC][VMIN] = 1
    mode[CC][VTIME] = 0
    termios.tcsetattr(stdin_fd, termios.TCSAFLUSH, mode)

    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
  return ch


def print_keyboard_help():
  print(f"Keyboard Commands:\n{KEYBOARD_HELP}")


def keyboard_poll_thread(q):
  print_keyboard_help()
  while True:
    c = getch()
    cmd = KEY_COMMANDS.get(c)
    if cmd is None:
      print_keyboard_help()
      continue
    q.put(control_cmd_gen(cmd))
    if cmd == "quit":
      break


def timed_poll_thread(q, duration, bridge_process=None):
  """Headless replacement for the keyboard: run for `duration` seconds, then quit."""
  end = time.monotonic() + duration
  while time.monotonic() < end:
    if bridge_process is not None and not bridge_process.is_alive():
      return
    time.sleep(0.5)
  q.put(control_cmd_gen("quit"))
