# Phase 5: Clear-distance speed limit and close-range braking

## Context

The MPC brakes only for `radarState.leadOne` and `leadTwo` (`long_mpc.py:350-351`). With no lead it invents a fast lead 50 m ahead (`long_mpc.py:323-327`) and drives at cruise speed, which equates "no lead" with "clear road". This phase makes speed depend on the distance confirmed clear, adds a hard brake at close range, warns the driver, and keeps overtaking from the minimum gap possible.

Requires user approval before coding: shared code.

## Mechanism

| Layer | Trigger | Action |
|---|---|---|
| 1. Clear-distance speed limit | Always | `v_cruise_eff = min(v_cruise, v_max(d_clear))`, with `v_max = -a * t_r + sqrt((a * t_r)² + 2 * a * (d_clear - d_stop))`, `a = COMFORT_BRAKE`, `t_r = 0.35 s`. Changes are ramped at no more than 1 m/s² upward so the car does not surge |
| 2. Approach deceleration | A lead exists | Existing MPC, fed the conservative `dRel` from phase 3 |
| 3. Close-range brake | In-path `d_clear` below `v_ego² / (2 * 3.5) + 2 m` | Force the plan to the maximum allowed deceleration until stopped; hold at standstill |
| 4. Driver warning | Layer 3 active, or layer 1 holding speed more than 10 km/h under set speed for an obstacle | FCW-style alert for layer 3; a "slow, obstacle ahead, steer manually" prompt for layer 1 |

`d_clear = min(conservative lead distance, clear-distance source)`. When the source is invalid or absent, its value is the fallback `SEE_RANGE_FALLBACK = 25 m`, which gives about 29 km/h (8.0 m/s with `d_stop = 6.4 m`). A valid source reporting 100 m or more allows full set speed.

## Overtaking from the minimum gap

| Rule | Behaviour | Why |
|---|---|---|
| In-path only | Layers 1 and 3 count an object only while it overlaps the car's predicted path (car half-width plus 0.3 m, using the model's path and the object's lateral offset). When the driver steers out, the object leaves the path and the limit lifts | Otherwise steering out while closing would trigger the hard brake |
| Driver throttle wins | Pressing the accelerator overrides layers 1 and 3; the warning stays on | Standard openpilot behaviour; the driver is responsible while overriding |
| Creep allowance | Below 5 km/h, layer 3's threshold drops to 1.5 m, so the car can turn out from a 4 m standstill gap | A 4 m hold would leave no room to manoeuvre |

The lateral offset of a vision lead (`yRel`) has not been validated at 4 to 10 m. The overtake scenario in the benchmark measures it before this rule is trusted; if `yRel` is too noisy, the in-path test uses the clear-distance source's corridor instead.

## Files

- Edit `selfdrive/controls/lib/longitudinal_planner.py`: subscribe to `clearDistance`, compute `d_clear` and `v_cruise_eff` (layer 1), close-range override (layer 3), in-path test, throttle override, creep allowance.
- Edit `selfdrive/controls/plannerd.py`: add `clearDistance` to the `SubMaster`.
- Edit `selfdrive/controls/controlsd.py` and `selfdrive/controls/lib/events.py`: the two alerts (layer 4).
- Edit `cereal/log.capnp` only if `longitudinalPlan` needs a field for the override state.
- Add `selfdrive/controls/tests/test_clear_distance_limit.py`.
- Flags: `ClearDistanceSpeedLimit`, `CloseRangeBrake`.

## Steps

1. Confirm in code how `v_cruise` reaches the MPC (`longitudinal_planner.py:63-91`) and where an acceleration override is safe with respect to `prev_accel_constraint` and the car's minimum acceleration.
2. Write unit tests first: `v_max` against `d_clear` (25 m gives about 8.0 m/s, 100 m gives about 18.7 m/s), upward ramp limit, layer 3 trigger distance at several speeds, no trigger for an out-of-path object, throttle override, creep threshold, fallback when the source is invalid.
3. Implement layer 3 and its alert, and benchmark it alone. It is the direct answer to "do not drive straight into it".
4. Implement layer 1 with the fallback only, benchmark; then with the phase 4 source, benchmark.
5. Run the overtake scenario and settle the in-path test.
6. Present results. The user picks flag defaults.

## Validation

- Unit tests and existing control tests pass.
- Flags off: baseline unchanged.
- Every row of the acceptance table in `plan.md` is measured and reported with the fallback only and with the phase 4 source.
- Peak deceleration never below the car's configured limit.

## Risk and rollback

- With no working source, the car is capped near 29 km/h whenever no lead is confirmed. That is the stated cost of not knowing the road is clear.
- A hard brake on a false obstacle is a rear-end risk. The clear-road and following scenarios must show 0 activations before any default is turned on.
- Rollback: flags off.
