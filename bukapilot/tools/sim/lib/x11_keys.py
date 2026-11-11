"""Read the keyboard while the CARLA window has input focus (X11, Linux Docker setup).

CARLA's window is a native X11 window of the carla_server container; its keys never reach Python. The
bridge container shares the host X server (DISPLAY + /tmp/.X11-unix), so the global key state can be
polled with XQueryKeymap (no key grab: CARLA and every other window still get their keys) and acted on
only while the focused window, or one of its parents, is the CARLA window.

Only libX11 through ctypes is used (libx11-6 is already in the bridge image).
"""
import ctypes
import ctypes.util
import os
import time

# Comma separated. "=UI" matches a window titled exactly UI (openpilot's UI window); other entries match as substrings.
DEFAULT_WINDOW_TITLE = "CarlaUE4,=UI"
POLL_PERIOD = 0.02    # s between key state reads
REPEAT_PERIOD = 0.1   # s between re-sent drive commands while a drive key is held; below MANUAL_TIMEOUT (0.25 s)
HOLD_KEYS = frozenset("wasd")


def pressed_keycodes(keymap):
  """Set of keycodes down in a 32 byte XQueryKeymap bit vector."""
  return {i * 8 + bit for i, byte in enumerate(keymap) for bit in range(8) if byte >> bit & 1}


def key_events(prev_down, cur_down, focused, last_sent, now, hold_keys=HOLD_KEYS):
  """Decide which characters to send for one poll.

  prev_down / cur_down are sets of characters down before and now (taken from the raw key state even when
  the CARLA window is not focused, so a key already down when focus arrives is not a new press).
  Returns (chars to send, updated last_sent dict). Nothing is sent without focus.
  """
  if not focused:
    return [], {}
  out = []
  sent = {}
  for ch in sorted(cur_down):
    if ch in hold_keys:
      if now - last_sent.get(ch, float("-inf")) >= REPEAT_PERIOD:
        out.append(ch)
        sent[ch] = now
      else:
        sent[ch] = last_sent[ch]
    elif ch not in prev_down:
      out.append(ch)
  return out, sent


class X11Keys:
  """Thin libX11 wrapper: key state of the characters we care about and CARLA-window focus."""

  def __init__(self, chars, title=None):
    lib = ctypes.util.find_library("X11") or "libX11.so.6"
    self.x = ctypes.CDLL(lib)
    x = self.x
    x.XOpenDisplay.restype = ctypes.c_void_p
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    x.XKeysymToKeycode.restype = ctypes.c_ubyte
    x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x.XGetInputFocus.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int)]
    x.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong),
                             ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_void_p),
                             ctypes.POINTER(ctypes.c_uint)]
    x.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_void_p)]
    x.XFree.argtypes = [ctypes.c_void_p]

    self.display = x.XOpenDisplay(None)
    if not self.display:
      raise RuntimeError("cannot open the X display (DISPLAY not set or not reachable)")
    # for ASCII characters the X keysym equals the character code
    self.keycode_to_char = {}
    for ch in chars:
      code = x.XKeysymToKeycode(self.display, ord(ch))
      if code:
        self.keycode_to_char[int(code)] = ch
    self.titles = [t.strip().lower() for t in (title or DEFAULT_WINDOW_TITLE).split(",") if t.strip()]

  def down_chars(self):
    keymap = ctypes.create_string_buffer(32)
    self.x.XQueryKeymap(self.display, keymap)
    return {self.keycode_to_char[c] for c in pressed_keycodes(keymap.raw) if c in self.keycode_to_char}

  def _name(self, window):
    name = ctypes.c_void_p()
    if self.x.XFetchName(self.display, window, ctypes.byref(name)) and name.value:
      text = ctypes.string_at(name.value).decode("utf-8", "replace")
      self.x.XFree(name)
      return text
    return ""

  def title_matches(self, name):
    name = name.strip().lower()
    return any(name == t[1:] if t.startswith("=") else t in name for t in self.titles)

  def carla_focused(self):
    """True if the focused window, or one of its parents, has a watched title. Fails closed."""
    window = ctypes.c_ulong()
    revert = ctypes.c_int()
    self.x.XGetInputFocus(self.display, ctypes.byref(window), ctypes.byref(revert))
    w = window.value
    for _ in range(8):  # the focus is often a child of the titled top-level window
      if w <= 1:  # None (0) or PointerRoot (1)
        return False
      if self.title_matches(self._name(w)):
        return True
      root, parent = ctypes.c_ulong(), ctypes.c_ulong()
      children, n = ctypes.c_void_p(), ctypes.c_uint()
      if not self.x.XQueryTree(self.display, w, ctypes.byref(root), ctypes.byref(parent),
                               ctypes.byref(children), ctypes.byref(n)):
        return False
      if children.value:
        self.x.XFree(children)
      if parent.value == 0 or parent.value == root.value:
        return False
      w = parent.value
    return False


def x11_key_poll_thread(q):
  """Send KEY_COMMANDS for keys pressed while the CARLA window has focus. Returns quietly if unavailable."""
  from tools.sim.bridge.common import control_cmd_gen
  from tools.sim.lib.keyboard_ctrl import KEY_COMMANDS

  title = os.environ.get("CARLA_WINDOW_TITLE", DEFAULT_WINDOW_TITLE)
  try:
    keys = X11Keys(KEY_COMMANDS.keys(), title)
  except (OSError, RuntimeError) as exc:
    print(f"keys in the CARLA window are off: {exc}. The terminal keyboard still works.", flush=True)
    return
  print(f"keys work while a window titled '{title}' has focus (CARLA, openpilot UI); the terminal keyboard works too.",
        flush=True)

  prev_down, last_sent = set(), {}
  while True:
    cur_down = keys.down_chars()
    chars, last_sent = key_events(prev_down, cur_down, keys.carla_focused(), last_sent, time.monotonic())
    prev_down = cur_down
    for ch in chars:
      cmd = KEY_COMMANDS[ch]
      q.put(control_cmd_gen(cmd))
      if cmd == "quit":
        return
    time.sleep(POLL_PERIOD)
