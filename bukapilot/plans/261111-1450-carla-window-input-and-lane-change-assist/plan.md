---
title: CARLA window input and blinker lane-change assist
status: implemented, simulator run pending
mode: fast
created: 2026-11-11
platform: tools/sim-linux-docker (Linux Docker, X11)
validated: 2026-11-11 (session 1)
blockedBy: []
blocks: []
---

# CARLA window input and blinker lane-change assist

## Outcome

1. Keys pressed while the **CARLA window is focused** control the car. The terminal no longer has to be focused.
2. When the user turns on the left or right blinker and gives a steering hint in the same direction, the car completes the lane change through the existing Assisted Lane Change (ALC), instead of the keyboard overriding openpilot's steering.
3. Like a normal lane change, the car first opens a larger gap behind the slow vehicle, then accelerates while changing lane. It does not depend on cruising fast beforehand. Today the following gap (13 to 17 m at 20 to 45 km/h) is too small for this.

## Constraints

- Target is the Linux Docker setup (`tools/sim-linux-docker`). The CARLA window is an X11 window from the `carla_server` container; the bridge container already has `DISPLAY` and `/tmp/.X11-unix`.
- Terminal keyboard input keeps working as it does today.
- Real-car control code stays unchanged by default; every `selfdrive/` change is opt-in through a sim-only environment variable. Any change in `selfdrive/` must keep today's value unless a sim-only environment variable is set.
- Python 3.8 inside the bridge image. No new pip dependency: use `ctypes` with `libx11-6`, which the image already installs.

## Non-goals

- Windows/WSL2 backend (`tools/sim/run.sh`, `run.ps1`).
- Automatic overtake decision (no blinker needed). Rear-traffic awareness in the old lane. Left-lane blindspot sensing from CARLA. Returning to the original lane after overtaking.
- Joystick path.

## Acceptance criteria

- With the terminal unfocused and the CARLA window focused, `1/2/3`, `z/x`, `w/a/s/d`, `r`, `i`, `h`, `q` do what the help table says. Keys typed in other windows do nothing.
- Press `z` once: the left blinker stays on (visible in carState / UI) until `z` is pressed again or the lane change finishes.
- With the blinker on and engaged above the lane change speed, pressing `a` once gives `laneChangeState` `preLaneChange` then `laneChangeStarting`, and the car physically changes lane via openpilot's own steering command. Pressing `a` without the blinker still steers manually as before.
- The same works behind a motorcycle at 60 km/h with the car's set speed at 80 km/h or more. `LANE_CHANGE_SPEED_MIN` stays unchanged (30 mph).
- With `ALC_GAP_TF=3.0`, after the blinker is on the car settles about 39 m behind a 60 km/h motorcycle, against about 16 m today (both from the gap formula at equal speed; the first run decides the final figure) before the lane change starts, and keeps accelerating while crossing, with no forward collision warning. With the variable unset, longitudinal behaviour is unchanged.
- Unit tests for key-state diffing, blinker latch and nudge routing pass; existing `tests/test_sim_carla.py` still passes.

## Phases

| # | Phase | Files | Depends on |
|---|---|---|---|
| 1 | [CARLA-window key capture](phase-01-carla-window-key-capture.md) | `lib/x11_keys.py` (new), `lib/keyboard_ctrl.py`, `run_bridge.py` | none |
| 2 | [Blinker latch and steering nudge](phase-02-blinker-latch-and-steering-nudge.md) | `bridge/common.py`, `lib/keyboard_ctrl.py` | none (can run alongside 1) |
| 3 | [Scenario speeds, docs, tests](phase-03-scenario-speeds-docs-tests.md) | `sim-linux-docker/demo.md`, `tests/` | 1, 2 |
| 4 | [Lane-change gap and acceleration](phase-04-lane-change-gap-and-acceleration.md) | `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`, `longitudinal_planner.py`, `plannerd.py`, `sim-linux-docker/start_docker.sh`, `demo.md` | 2 |

All paths are relative to `bukapilot/tools/sim` unless they start with `selfdrive/` or `sim-linux-docker/`.

## Decision made

Speed gate: the shared constant stays at 30 mph (about 48.3 km/h). Instead the scenario is run faster, with the motorcycle at 60 km/h and the car's set speed at 80 km/h or more (user decision, temporary). Phase 3 therefore makes no `selfdrive/` change; the only `selfdrive/` edits in the whole plan are the opt-in ones in phase 4.

## Risks

- Focus detection depends on the CARLA window's X11 title; verify the real title in phase 1 before coding the match.
- Wayland sessions: `XQueryKeymap` only reports keys while an XWayland window has focus, which is the CARLA case, but behaviour should be confirmed on the user's machine.
- The motorcycle must actually reach 60 km/h in CARLA; phase 3 checks this on the first run.

## Validation Log

### Session 1 (2026-11-11)

**Verification Results**
- **Tier:** Standard (3 phases; Fact Checker + Contract Verifier)
- **Claims checked:** 27
- **Verified:** 24 | **Failed:** 3 | **Unverified:** 0 (the 3 failures were corrected in the phase files; they were wrong plan text, not wrong design)

#### Failures (all corrected)
1. [Fact Checker] `desire_helper.py:11` for `LANE_CHANGE_SPEED_MIN`: actual line is 12 (`LANE_CHANGE_TIME_MAX` 13, `ALC_CANCEL_DELAY` 14). Phase 3 corrected.
2. [Fact Checker] Phase 2 assumed the bridge already subscribes to `lateralPlan`: `bridge/common.py:200` lists only `carState`, `carControl`, `controlsState`. Phase 2 now adds it explicitly.
3. [Fact Checker] Phase 3 named `pytest`; the repo's offline tests run with `unittest discover` (see `tests/test_sim_carla.py` docstring). Corrected.

#### Verified highlights
- `run_bridge.py:96-103` terminal keyboard path; `KEY_COMMANDS` has no other consumer, so reusing it is safe.
- `libx11-6` is in `Dockerfile.bukapilot`; `start_docker.sh` passes `DISPLAY`, mounts `/tmp/.X11-unix`, and runs `xhost +local:`.
- Torque sign and size: `a` sends `steer_-0.15`, `user_torque = -0.15 * -10000 = +1500`; Honda `steeringTorque` is the raw sensor value and the Civic threshold is 1200.
- Contract: `LANE_CHANGE_SPEED_MIN` has 7 consumers (`desire_helper.py`, `controlsd.py:30,493`, byd/perodua/proton car files); a module-level env override applies to all consistently and defaults to today's value.
- Env propagation: `launch_openpilot.sh` runs inside the bridge container, so only the `docker run -e` list needs the new variable.

**Questions and answers**
1. Speed gate approach: not chosen. The user asked if 48 km/h is an upper or lower bound (answer: lower bound). Plan keeps the sim-only override as the working default; confirm before cooking phase 3.
2. Blinker after lane change: turns off automatically when the lane change ends (confirmed). Manual `z`/`x` still toggles it. Phase 2.
3. Blindspot from CARLA: out of scope (confirmed). `demo.md` notes the user must check the left lane.

### Whole-Plan Consistency Sweep
- Files reread: plan.md, phase-01, phase-02, phase-03
- Decision deltas checked: 3
- Reconciled stale references: 3
- Unresolved contradictions: 0 (one open decision: speed gate)

### Session 2 (2026-11-11), phase 4 decisions
1. Starting gap: 3.0 s time headway (`ALC_GAP_TF` default), to be tuned in the simulator run.
2. Behaviour: only open the gap before the lane change; the lane change is not blocked until the gap is reached.
3. Speed gate (phase 3): undecided at that point; resolved in session 3.

Phase 4 has not yet gone through the verification roles; its file and line claims come from the scout in this session (`long_mpc.py:80-84,332-344`, `longitudinal_planner.py:19-21`, `plannerd.py:41-45`).

### Session 3 (2026-11-11), speed gate decision
The user chose to raise the car and motorcycle speeds temporarily instead of changing any threshold. Phase 3 was rewritten (renamed `phase-03-scenario-speeds-docs-tests.md`); `ALC_MIN_SPEED_KPH` is dropped. Phase 4 no longer depends on phase 3. Open item: whether CARLA lets the motorcycle exceed the road speed limit (checked in the first run).

### Session 4 (2026-11-11), phase 4 validation
**Verification Results**
- **Tier:** Standard (Fact Checker + Contract Verifier), phase 4 only
- **Claims checked:** 14
- **Verified:** 10 | **Failed:** 4 | **Unverified:** 0 (all 4 corrected in phase 4)

#### Failures (corrected)
1. [Fact Checker] `long_mpc.py:332-340` for the lead use: actual `343-352`, and it covers `leadOne` and `leadTwo` plus `status`, the braking limit and the crash counter. Requirement 2 now ignores all of them.
2. [Fact Checker] `plannerd.py:41-45` should be `38-42`.
3. [Fact Checker] "braking stays within `A_CRUISE_MIN`" is wrong when a lead exists: the MPC limit is `MIN_ACCEL` -3.5 m/s². Wording corrected; peak deceleration is now recorded in the simulator run.
4. [Contract Verifier] `Planner.update(sm)` has two callers (`plannerd.py:41`, `selfdrive/test/longitudinal_maneuvers/plant.py:48`) and `LongitudinalMpc.update` one (`longitudinal_planner.py:91`). New arguments must be optional so the maneuver test plant is unaffected.

#### Verified
`long_mpc.py:61,63,80-84,344,370`, `longitudinal_planner.py:19-21`, `bridge/common.py:56`, `lateral_planner.py:17` (`self.DH`), and the loop order lateral-then-longitudinal in `plannerd.py`.

**Question and answer**
- Lead handling while crossing: ignore the lead except below the 1.0 s floor (recommended option, confirmed by the user).

### Whole-Plan Consistency Sweep
- Files reread: plan.md, phase-01 to phase-04
- Decision deltas checked: 4
- Reconciled stale references: 4
- Unresolved contradictions: 0

## Implementation Log (2026-11-11)

All four phases are implemented and not committed. Offline checks passed inside the running `bukapilot_sim_carla` container: 26 unit tests (existing plus `tools/sim/tests/test_sim_input.py`), the 6 longitudinal maneuver tests with the default environment and with `ALC_GAP_TF=3.0`, and a live X11 read (13 of 13 keys mapped, the CARLA window title `CarlaUE4` matches, the poll thread starts). A direct probe of the MPC at 60 km/h confirmed the intended effect: with the lead ignored the car may accelerate up to +1.20 m/s² against +0.11 with the lead; a lead closer than the 1.0 s floor is kept; a 3.0 s headway gives a 39.0 m gap against 15.7 m.

Not done: the end-to-end scenario run (blinker, nudge, lane change past a 60 km/h motorcycle) and its report, because that needs the user's running CARLA session to be restarted. The longitudinal maneuver test needs `StoppingDistanceOffset` in the params store; it fails the same way on the original code, so it was run with that parameter faked.
