# Phase 4: Lane-change gap and acceleration

## Context

User requirement: a normal lane change keeps a large enough gap first, then accelerates during the change. It does not rely on cruising fast beforehand. Today the gap is too small for that.

Evidence from the code:

- Desired gap is `v²/(2·2.0) + tf·v + 6.4 − v_lead²/(2·1.6)` (`long_mpc.py:80-84`). The time headway `tf` comes from the cruise distance setting (`long_mpc.py:344`); `normal` is 1.6 s (`long_mpc.py:61`).
- At the same speed as the lead, that is about 13.4 m at 20 km/h, 16.0 m at 35 km/h and 16.6 m at 45 km/h. With `tf` 3.0 s it would be about 21, 30 and 34 m.
- The planner cannot accelerate through the lane change. The motorcycle is still the in-path lead until the car has crossed, so the MPC keeps holding the gap (`long_mpc.py:332-340` uses `leadOne` directly).
- Acceleration is capped at 1.2 m/s² at low speed, falling to 0.6 at 40 m/s (`longitudinal_planner.py:19-21`). The simulator's own limit is 1.6 m/s² (`bridge/common.py:56`), so it does not bind. Over a 4 to 5 s lane change that cap closes only about 10 to 15 m (½·a·t²), so the starting gap has to carry most of the clearance. These are estimates to calibrate in the simulator.
- `plannerd.py:41-45` runs the lateral planner and then the longitudinal planner in the same loop, so the lane change state (`lateral_planner.DH.lane_change_state`) can be passed in with no new subscription.

## Requirements

1. **Open the gap before the change.** While one blinker is on, ALC is enabled, the car is engaged, and the lane change state is `off` or `preLaneChange`, the MPC uses `ALC_GAP_TF` (default 3.0 s, confirmed by the user as the starting value) instead of the normal headway. The gap is only opened; the lane change is not gated on reaching it, so a nudge before the gap is open still starts the change (confirmed by the user). Ramp the headway at a limited rate (about 0.3 s per second) so braking stays within `A_CRUISE_MIN`.
2. **Accelerate through the change.** While the state is `laneChangeStarting` or `laneChangeFinishing`, do not let the in-path lead limit acceleration, except below a hard safety floor: if the lead is closer than the `tf` 1.0 s distance (`T_FOLLOW_DANGER`), keep the lead.
3. **Return smoothly.** After `laneChangeFinishing` ends or the blinker goes off, ramp the headway back to normal over about 2 s.
4. **No gating.** Do not delay `preLaneChange` or `laneChangeStarting` until the gap is reached.
5. **Disabled by default.** With `ALC_GAP_TF` unset, behaviour is exactly today's. This code is in `selfdrive/`, shared with real cars, so it is opt-in through the environment, like the speed gate in phase 3.

## Files

- Edit `selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`: accept a headway override and a "ignore in-path lead" flag in `update`; keep the logic in one small pure function so it can be unit-tested.
- Edit `selfdrive/controls/lib/longitudinal_planner.py`: take the lane change state and blinker flag, call the pure function, pass the result to the MPC.
- Edit `selfdrive/controls/plannerd.py`: pass `lateral_planner.DH.lane_change_state` into `longitudinal_planner.update`. Blinker and engaged state come from `carState` and `controlsState`, already subscribed.
- Edit `tools/sim-linux-docker/start_docker.sh` (add `-e ALC_GAP_TF`), `run.sh` header, and `demo.md`.
- Add tests to `tools/sim/tests/test_sim_input.py` (or a sibling file) for the pure function.

## Steps

1. Write the pure function first: inputs are state, blinker, engaged, env value, previous headway; output is headway and the ignore-lead flag. Test: disabled when unset, large headway in `preLaneChange`, rate limit, ignore-lead only in `laneChangeStarting`/`Finishing`, safety floor, ramp back.
2. Wire it into the planner and MPC without changing any code path when the variable is unset.
3. Forward the variable and document it.
4. Simulator run behind the 60 km/h motorcycle with phases 1 to 3 in place, using the 60 km/h scenario from phase 3. Record in a report under `plans/reports/`: gap when `laneChangeStarting` begins, minimum gap during the crossing, peak acceleration, and whether a forward collision warning appeared.

## Validation

- Offline unit tests for the pure function (`unittest discover`, same command as phase 3).
- Scenario: `ALC_GAP_TF=3.0 bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle-spec "same:40:60"` with the set speed at 80 km/h or more. Expect the car to fall back to a larger gap behind the 60 km/h bike after the blinker is on, change lane on the nudge, and accelerate while crossing.

## Risks and rollback

- Slowing down to open the gap works against the pass; the car may sit low in speed for several seconds. The 3.0 s default is a starting value to tune, not a measured optimum.
- Ignoring the lead during the crossing is the riskiest part. The safety floor and the opt-in flag limit it; if the floor proves too weak in the run, keep the lead and only raise the acceleration limit.
- Another vehicle close behind, in the old lane, is not considered (no rear radar in the sim).
- Rollback: unset `ALC_GAP_TF`, or revert the three `selfdrive/` edits.
