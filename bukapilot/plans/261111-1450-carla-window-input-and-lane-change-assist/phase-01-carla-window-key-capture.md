# Phase 1: CARLA-window key capture

## Context

`run_bridge.py:96-103` reads keys with `keyboard_poll_thread`, which blocks on terminal stdin (`keyboard_ctrl.py:getch`). CARLA's window cannot deliver keys to Python. In the Docker setup both containers share the host X11 server, so the bridge can read the global key state and act only while the CARLA window has input focus.

## Requirements

- Poll key state at about 50 Hz with `XQueryKeymap`; no key grab, so CARLA and other windows still receive their keys.
- Act only when the focused window, or one of its ancestors, has a title containing `CARLA_WINDOW_TITLE` (default taken from the verified real title, expected `CarlaUE4`).
- Reuse `KEY_COMMANDS` from `keyboard_ctrl.py`; do not duplicate the mapping.
- Toggle keys (`1 2 3 z x i h r q`) fire once per press edge. Drive keys (`w a s d`) re-send their command every 100 ms while held, which is under `MANUAL_TIMEOUT` (0.25 s), and the release is detected from the key state instead of a timeout guess.
- Terminal input stays active. `CARLA_KEYS=0` or `--no-carla-keys` turns the new path off. If `libX11` or `DISPLAY` is missing, print one line and continue with the terminal only.

## Files

- Create `lib/x11_keys.py`: ctypes bindings (`XOpenDisplay`, `XQueryKeymap`, `XKeysymToKeycode`, `XGetInputFocus`, `XQueryTree`, `XFetchName`), a pure function that diffs two keymaps into press/release events, and `x11_key_poll_thread(q)`.
- Edit `lib/keyboard_ctrl.py`: update the help text to say keys work in the CARLA window.
- Edit `run_bridge.py`: start the X11 thread as a daemon thread before the blocking terminal loop.

## Steps

1. With CARLA running, read the real window title (`xprop -id $(xdotool getwindowfocus)` or `xwininfo -root -tree`) and record it as the default. This settles the open risk before coding.
2. Write the pure keymap-diff function and its unit tests first.
3. Write the ctypes layer and focus check; keep all X calls in one module.
4. Wire into `run_bridge.py`; keep `--duration` and `--joystick` paths untouched.

## Validation

- Unit tests: diff of two 32-byte keymaps returns the expected press/release sets.
- Manual: run `bash run.sh all ...`, click the CARLA window, press `1` then `2` and watch cruise change in the bridge log; click the terminal and confirm CARLA-only keys no longer double-fire.

## Risks and rollback

- Both terminal and CARLA input active at once could double-send a key if the terminal is focused while the title check fails open. Fail closed: no match, no event.
- Rollback: `CARLA_KEYS=0`, or remove the thread start in `run_bridge.py`.
