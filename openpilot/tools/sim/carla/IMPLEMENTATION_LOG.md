# CARLA bridge implementation log

> **Historical record.** Entries describe work done on the pre-restructure
> layout, where `selfdrive/`, `system/` and `tools/` sat at the repository root.
> That source now lives under `openpilot/`. Paths are left as written at the
> time; read them as history, not as a map of the current tree.

Date: 2026-08-05

Baseline: openpilot `v0.11.1` (`4df40d2`), CARLA `0.9.16`.

## 2026-08-05 handoff cleanup

- Removed unused `tools/sim/wsl2/.state` containing the legacy CARLA 0.9.13
  runtime (31,533 files, 19.02 GiB). It is not recoverable without downloading
  that old CARLA package again.
- Removed the separate `run.sh openpilot`/PID workflow; the CARLA bridge is the
  single owner of manager in the supported all-in-one process tree.
- Made the WSL launcher derive its repository path instead of hardcoding
  `/mnt/d/work/openpilot`.
- Made the firewall rule resolve the selected WSL distro's current IPv4 `/32`
  instead of hardcoding one machine's WSL subnet.
- Replaced the stale README instructions with a concise overview and added the
  contributor-facing `installation.md` guide.

## 2026-08-06 codebase cleanup and beginner documentation

- Removed `.legacy-v094-submodules/` after confirming the current source has no
  references to it. The separately preserved Git stash was not removed.
- Removed packaging scratch directories `.carla/export-staging` and
  `.carla/export-verify`, plus `tools/sim/__pycache__`.
- Kept `.carla/exports` because it contains the developer handoff archive.
- Added `CONNECTION_WALKTHROUGH_VI.md`, explaining the complete Windows/WSL,
  RPC, camera, CAN, message-bus, engagement, and actuator path for beginners.

## Storage policy

- Repository, virtualenv, CARLA download/install, Windows profile, temp files,
  caches, and logs are rooted under `D:\work\openpilot`.
- WSL uses `/mnt/d/work/openpilot`.
- Linux-only caches use the Ubuntu VHD stored at `D:\wsl\ubuntu`.
- No project download or cache is intentionally rooted on `C:`.

## Source changes

- Added the CARLA world/bridge under `tools/sim/bridge/carla`.
- Added a chase spectator camera and made interactive WSL runs request a visible
  CARLA renderer by default (`CARLA_HEADLESS=1` retains offscreen mode).
- The WSL launcher prepends `.carla/bin` itself, so `uv` remains available in
  fresh non-login shells after a WSL restart.
- Interactive bridge output is mirrored to `.carla/logs/bridge.log` while
  keyboard input remains attached to the terminal.
- `stop` also cleans stale manager/CARLA-bridge children from vanished terminal
  sessions; matching is restricted to this repo's manager and CARLA command.
- Manual WASD commands persist for 250 ms (and decay automatically), while
  startup cruise button phases persist for 100 ms so the CAN thread observes
  engagement reliably.
- Superseded during UI integration: manager initially used `CI=1`; the final
  launcher enables the WSLg UI by default and `OPENPILOT_UI=0` disables it.
- Road and wide camera frames are converted first and then published with one
  monotonic timestamp, preventing modeld `frames out of sync` drops.
- The all-in-one bridge owns the manager subprocess in the same process tree as
  the closed-loop test. Exiting the keyboard bridge also cleans up manager and
  prevents detached manager/world conflicts.
- CARLA applies a 0.25 startup throttle only until the first openpilot
  engagement held continuously for 3 seconds, then hands longitudinal control
  entirely to openpilot; transient engagement resumes the startup roll.
- Added Windows and WSL launchers plus documentation under `tools/sim/carla`.
- Added CARLA selection/options to `tools/sim/run_bridge.py`.
- Added the optional CARLA dependency to `pyproject.toml` and updated `uv.lock`.
- Added `.carla/` to `.gitignore`.
- Updated `tools/sim/launch_openpilot.sh` to use the repo virtualenv and explicit
  import paths on DrvFS.
- Updated simulated camerad to use device-compatible padded NV12 buffers and
  monotonic timestamps.
- Updated modeld to copy simulator CPU VisionIPC buffers to the CUDA device.
- Added `tools/sim/tests/test_carla_bridge.py`.

## System change

The original Windows Firewall rule permitted inbound TCP 2000-2002 for the
CARLA executable and WSL subnet `172.18.176.0/20`. The final portable launcher
resolves the selected distro's current IPv4 address and creates a `/32` rule.
Every attempted add/remove is recorded at:

`D:\work\openpilot\.carla\firewall-events.jsonl`

Remove the rule from an Administrator PowerShell:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File D:\work\openpilot\tools\sim\carla\run.ps1 -Command firewall-remove
```

## Source revert

The following restores modified tracked files to the v0.11.1 baseline:

```bash
git restore .gitignore pyproject.toml uv.lock \
  selfdrive/modeld/modeld.py \
  tools/sim/launch_openpilot.sh tools/sim/lib/camerad.py tools/sim/run_bridge.py
```

After reviewing that they contain only this implementation, remove the added
paths `tools/sim/bridge/carla`, `tools/sim/carla`, and
`tools/sim/tests/test_carla_bridge.py`. Runtime data is isolated under
`D:\work\openpilot\.carla`; remove it separately only if the CARLA download,
installation, logs, and audit history are no longer needed.

The earlier v0.9.4 attempt remains separately preserved in Git stash
`codex-v0.9.4-carla-attempt` and `.legacy-v094-submodules/`.
# 2026-08-05: Clean up namespaced openpilot orphans

- Symptom: `manager.py` and the bridge were gone, but old `selfdrive.modeld.modeld`,
  `selfdrive.ui.feedback.feedbackd`, `system.statsd`, and
  `system.loggerd.deleter` processes remained under WSL `/init` sessions.
- Cause: managed processes replace their command line with a Python module name,
  so the previous cleanup match containing the repository path could not find
  them after their manager exited.
- Change: `run.sh stop` now terminates only process titles beginning with the
  openpilot namespaces `selfdrive.` or `system.`, using TERM followed by KILL.
- Revert: remove the two namespaced `pkill` lines for TERM and KILL from
  `stop_openpilot` in `tools/sim/carla/run.sh`.

# 2026-08-05: Restore simulator driving and clear stale safety alert

- Symptom: openpilot launched but manager reported
  `no_excessive_actuation=false`; after clearing it manually, openpilot engaged
  but could command zero throttle and the ego vehicle stopped.
- Change: `run_bridge.py --launch-openpilot` removes only the simulator-stale
  `Offroad_ExcessiveActuation` Param before manager starts.
- Change: active keyboard/joystick input now overrides openpilot actuators even
  while engaged, so `W/S/A/D` remain usable as an explicit simulator override.
- Revert: remove the Params cleanup in `tools/sim/run_bridge.py` and the
  `manual_control_active` output-selection block in
  `tools/sim/bridge/common.py`.

# 2026-08-05: Emulate stock longitudinal control correctly

- Root cause of the ego car braking to a stop after engagement:
  `carParams.openpilotLongitudinalControl` is false for the simulated Honda,
  and the bridge incorrectly converted the inactive `accel=-3.5` sentinel into
  a real brake command.
- Change: apply `carControl.actuators.accel` only when
  `openpilotLongitudinalControl` is true. Otherwise emulate the Honda's missing
  stock ACC with a bounded proportional speed controller targeting 8 m/s;
  openpilot continues to provide lateral control.
- Revert: remove the `openpilotLongitudinalControl` branch in
  `tools/sim/bridge/common.py` and `stock_cruise_speed` in
  `tools/sim/bridge/carla/carla_bridge.py`.

# 2026-08-05: Show the openpilot UI by default

- Root cause of the missing UI: `run.sh` defaulted to `CI=1`, and
  `launch_openpilot.sh` blocks `ui` whenever `CI` is set. `run_bridge.py` also
  restored `CI=1` when the variable had deliberately been unset.
- Change: visible CARLA runs now default to `OPENPILOT_UI=1`, leave `CI` unset,
  and pass that environment unchanged to manager. Use `OPENPILOT_UI=0` for a
  headless/integration run.
- Revert: restore the default in `run.sh` from `OPENPILOT_UI:-1` to
  `OPENPILOT_UI:-0`, and restore the manager `CI` default assignment in
  `tools/sim/run_bridge.py`.

- Diagnostic addition: bridge status now records the exact steer/throttle/brake
  sent to CARLA and whether `openpilotLongitudinalControl` is active. Revert by
  removing `last_controls`, `openpilot_longitudinal`, and the `Controls:` status
  line from `tools/sim/bridge/common.py`.

# 2026-08-05: Remove AlphaLongitudinal startup race

- Diagnostic result: failing runs showed `OP-long=True`, `accel=-3.5`, and the
  bridge applying `brake=0.88`; earlier runs showed `OP-long=False`. The outcome
  depended on whether `card` or bridge initialization read/wrote
  `AlphaLongitudinalEnabled` first.
- Change: set `AlphaLongitudinalEnabled=False` for CARLA before manager starts,
  and make `CarlaBridge` declare the same setting. The simulated Honda now
  deterministically uses bridge-emulated stock ACC at 8 m/s with openpilot
  lateral control. MetaDrive retains its existing alpha-longitudinal default.
- Revert: remove the pre-manager Param assignment in `run_bridge.py`, restore
  the unconditional `True` assignment in `SimulatorBridge`, and remove
  `alpha_longitudinal_enabled` from `CarlaBridge`.

# 2026-08-05: Break stock-ACC engagement deadlock

- Symptom after making stock longitudinal deterministic: the car rolled to
  4.91 m/s but openpilot never engaged, then the car reached an obstruction.
- Root cause: simulated CAN reported `ACC_STATUS` from
  `simulator_state.is_engaged`, while openpilot requires stock ACC enabled before
  it can become engaged.
- Change: cache `AlphaLongitudinalEnabled` in `SimulatedCar`; for stock-long
  simulation, report ACC enabled whenever ignition is on. Alpha-long behavior
  remains unchanged.
- Revert: remove `alpha_longitudinal_enabled` from `SimulatedCar` and restore
  `ACC_STATUS` to `int(simulator_state.is_engaged)`.

# 2026-08-05: Initialize openpilot before moving CARLA

- User-visible symptom: UI said `openpilot unavailable` while the ego car had
  already begun rolling.
- Change: CARLA now publishes stationary CAN/camera data while
  manager, card, modeld, controlsd, selfdrived, and UI initialize. Only after
  `selfdriveState` is actually alive (plus a 3-second minimum) does the bridge
  raise the simulated stock-ACC enable edge, issue cruise buttons, and apply
  startup throttle. This avoids relying on startup timing on `/mnt/d`.
- `SimulatorState.stock_cruise_enabled` now represents the independent stock
  ACC state; `SimulatedCar` reports this on CAN instead of enabling ACC before
  openpilot is ready.
- Revert: remove `openpilot_startup_delay` from `CarlaBridge`, the
  `openpilot_ready_time` gating in `SimulatorBridge`, and restore the previous
  stock `ACC_STATUS` expression in `SimulatedCar`.

- Follow-up: after `selfdriveState` becomes engageable, pulse stock ACC off/on
  every 0.5 seconds until `selfdriveState.active=True`, then hold ACC enabled.
  This guarantees a PCM cruise-enable edge is observed even if selfdrived comes
  alive between CAN frames. Revert by assigning
  `stock_cruise_enabled = openpilot_startup_ready` directly.

- Stability follow-up: latch the startup handshake after the first alive
  `selfdriveState`. A brief messaging `alive=False` sample must not return the
  running vehicle to startup state and disable stock ACC. Revert by recalculating
  `openpilot_startup_ready` directly from `sm.alive` on every loop.

# 2026-08-05: Synchronize simulated panda safety configuration

- Symptom: openpilot engaged and drove at about 6.05 m/s, then raised
  `controlsMismatch` and disengaged.
- Root cause: simulated pandaState hardcoded the `BOSCH_LONG` safety flag while
  CARLA was configured for stock longitudinal, so it stopped matching
  `carParams.safetyConfigs` after the 10-second safety grace period.
- Change: publish panda `safetyModel` and `safetyParam` from the current
  `carParams.safetyConfigs[0]`, with the old constants only as a pre-fingerprint
  fallback. After three seconds of sustained engagement, keep stock ACC enabled
  rather than resuming pulses after a transient disengagement.
- Revert: restore the hardcoded safety model/param in `SimulatedCar` and remove
  `past_startup_engaged` from the stock-cruise hold condition.

# 2026-08-05: Re-engage after crash or reset

- Symptom: after the first sustained engagement, resetting the CARLA vehicle
  could leave stock ACC continuously enabled, so selfdrived never observed a
  fresh PCM enable edge.
- Change: `r` now resets the world, clears the previous engagement handoff, and
  forces stock ACC off for 0.5 seconds before the normal auto-engage pulse.
- Pressing `2` while disengaged performs the same stock-ACC re-arm, allowing
  recovery after a crash without resetting the vehicle.
- Revert: remove `stock_cruise_rearm_frames` and its reset/cruise handling from
  `tools/sim/bridge/common.py`.
