# Phase 2: Hypothesis checks

## Context

Three things are unexplained and decide where the fix belongs (sim or shared code). None needs a `selfdrive/` change.

## Checks

| # | Hypothesis | Method | Result that confirms it |
|---|---|---|---|
| H1 | Town04's wrong lead speed (7 m/s for a stationary motorcycle) is caused by the scene, not by the earlier probe | Phase 1 benchmark in Town04 with a normally spawned motorcycle, ego at 8 m/s and engaged | Model `v` above 5 m/s and a lead only inside 15 m |
| H2 | The BGRA to YUV conversion feeding the model is numerically off | Convert one bridge frame with `tools/sim/lib/camerad.py` and with a reference BT.601 conversion; compare per-plane mean and maximum difference | Mean difference above 2 levels, or a range mismatch (16-235 against 0-255) |
| H3 | The errors come from CARLA images, not from the model itself | Replay a real-drive segment with a stopped or slow lead through the same `modeld`; compare `prob`, `x`, `v` against the log's radar lead | On real images the lead is held beyond 60 m with `v` within 2 m/s of radar |
| H4 | Slope or lighting explains Town04 | Repeat H1 on a flat Town04 stretch and under the Town05 weather preset | Result changes with slope or weather |

## Files

- Probes live in the scratchpad, or in `tools/sim/tests/manual/` if they are worth keeping. No production file changes.
- Edit `plans/Report/report.md` with each result.

## Steps

1. H2 first (about 10 minutes, the stack only needs to be up).
2. H1 and H4 with the phase 1 runner.
3. H3 is optional now that the scope is the simulator only: it tells whether the model itself or the CARLA images cause the errors, but no longer decides where the fix goes. Run it if a log is at hand (the user's, or a public comma segment); `tools/replay` feeds `modeld`.
4. Write the outcome per hypothesis and apply the decision gate in `plan.md`.

## Validation

Each hypothesis ends as confirmed, rejected or not testable, with the numbers that back it.

## Decision output

- H2 confirmed: fix the conversion in the sim camera path, rerun the baseline, and re-evaluate whether phases 3 and 4 are still needed at the same strength.
- H3 confirmed (real images are fine): record that the sim under-represents the model. Phase 3 part B is dropped. Phase 4 is still built, but its defaults are tuned conservatively and re-checked on the real car.
- H3 rejected (real images show the same errors): phases 3 and 4 proceed in full.
