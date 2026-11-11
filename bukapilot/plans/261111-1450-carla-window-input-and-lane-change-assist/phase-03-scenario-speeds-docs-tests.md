# Phase 3: Scenario speeds, docs, tests

## Context

ALC only starts at or above `LANE_CHANGE_SPEED_MIN`, 30 mph or about 48.3 km/h (`desire_helper.py:12`). Below it, a blinker also turns off lateral control (`controlsd.py:548,570-571`). The default motorcycle set (`same:35:20, same:75:45`) keeps the car under that. The user chose to leave the shared constant alone and raise the speeds in the scenario instead, temporarily.

Web search found no source that justifies lowering the threshold, and the openpilot FAQ gives the same 30 mph, so the constant stays. No `selfdrive/` change in this phase.

## Requirements

- The scenario keeps the car at or above about 55 km/h while it follows the motorcycle, including after phase 4 opens the gap, so it never dips under 48.3 km/h before the lane change starts.
- Recommended starting point: one motorcycle in the same lane at 60 km/h, and the car's set speed (`vCruise`) at least 80 km/h.
- No other bikes in the left lane for the first trials: the simulator reports no blindspot.

## Files

- Edit `sim-linux-docker/demo.md` (Vietnamese, matching the file): add the scenario below, the key table (keys work in the CARLA window; `z`/`x` latch the blinker; `a`/`d` nudge), and a note that ALC needs 48.3 km/h or more and that the left lane must be checked by eye.
- Add `tests/test_sim_input.py` as a `unittest` module following `tests/test_sim_carla.py` (used by phases 1, 2 and 4).

Scenario command (single bike, no left-lane bike; final numbers to be adjusted after the first run):

```bash
bash run.sh all --carla-town Town04_Opt --carla-spawn-point 170 --motorcycle-spec "same:40:60"
```

## Steps

1. Check how `vCruise` is set in the simulator (auto-engage sets it, `1` raises it) and read it from the bridge log line `vCruise=... kph`. Record how many presses reach 80 km/h.
2. Confirm the motorcycle reaches 60 km/h: `_update_motorcycle_speeds` (`carla_world.py:196-203`) converts the target to a percentage of the road speed limit and clips to -100..100, so a target above the limit should work; verify in the first run.
3. Write the demo section.
4. Run the full scenario once after phases 1, 2 and 4, and record in a report under `plans/reports/`: speed of the car at the blinker, ALC states reached, whether the lane change completed.

## Validation

- `PYTHONPATH=<bukapilot root> python3 -m unittest discover -s tools/sim/tests -t . -v` for new and existing offline tests.
- Scenario run from the acceptance criteria in `plan.md`.

## Risks and rollback

- If the motorcycle cannot exceed the road speed limit, use a faster road or a different spawn point; the fallback is the earlier option of a sim-only `ALC_MIN_SPEED_KPH` override, not a shared-constant change.
- Rollback: documentation and test changes only.
