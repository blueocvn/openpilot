# Phase 2: Blinker latch and steering nudge

## Context

- `bridge/common.py:155-157,227-233`: a blinker key sets `held_blinker` for `HELD_INPUT_FRAMES` (10) frames, then clears. ALC needs the blinker to stay on while the user nudges, and falls back to `off` when it drops.
- `bridge/common.py:259-262`: any manual steer sets `manual_control_active`, and the steering angle then comes from the key (`manual_steer * MANUAL_STEER_DEG`), so openpilot's lane change command is overridden.
- `bridge/common.py:237`: `user_torque = manual_steer * MANUAL_TORQUE`. With `a` = -0.15 and `MANUAL_TORQUE` = -10000 the torque is +1500, above the 1200 threshold and positive, which `desire_helper.py` treats as left. Honda `carstate.py:254` passes `STEER_TORQUE_SENSOR` straight through as `steeringTorque` and the Civic threshold is the default 1200, so the sign and size are consistent; keep a test for it.

## Requirements

- `z` / `x` latch the left / right blinker on and off (toggle). Pressing the other side switches sides. Cruise buttons keep the 10-frame pulse.
- Auto-clear the latch once a lane change that started has returned to `off`, so the blinker does not stay on after the car has changed lane. A manual toggle always wins.
- Nudge routing: when a blinker is latched, the car is engaged, and the manual steer sign matches the blinker side, send `user_torque` but keep `steer_out = steer_op` (openpilot steers). In every other case behave as today. Throttle and brake keys remain manual overrides.
- Constants (`ASSIST_*`) live next to the other class constants in `common.py`.

## Files

- Edit `bridge/common.py`: replace `held_blinker` use for blinkers with a latched `blinker` state, add the auto-clear by adding `'lateralPlan'` to the bridge's `SubMaster` list at `common.py:200` (it currently has only `carState`, `carControl`, `controlsState`), split the manual override into throttle/brake and steering parts.
- Edit `lib/keyboard_ctrl.py`: help text for latch behaviour.

## Steps

1. Extract the per-frame routing (blinker flags, `user_torque`, `steer_out`) into a small pure method so it can be tested without CARLA.
2. Add the latch and auto-clear.
3. Add the nudge branch.
   <!-- Updated: Validation Session 1 - blinker clears automatically after the lane change (confirmed by user) -->
4. Tests: latch toggle and side switch; nudge keeps `steer_out == steer_op`; no blinker keeps manual override; opposite direction keeps manual override; torque sign positive for left.

## Validation

- Unit tests above.
- Manual: engage, press `z`, press `a` once above the lane change speed, confirm `preLaneChange` then `laneChangeStarting` in the bridge log and that the car changes lane without holding `a`.

## Risks and rollback

- Auto-clear could clear too early if `laneChangeState` is `off` before the first nudge. Only clear after the state has left `off` at least once since the latch was set.
- Rollback: revert `common.py`; blinkers return to the 10-frame pulse.
