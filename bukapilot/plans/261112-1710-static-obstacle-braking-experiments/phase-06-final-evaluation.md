# Phase 6: Final evaluation and docs

## Context

Phases 3 to 5 each ran the short benchmark. This phase runs the full matrix once with the flag set the user chose, compares it with the phase 1 baseline, and documents the result.

## Files

- Edit `plans/Report/report.md`: final table per scenario, town and speed (baseline against final), what remains unsolved.
- Edit `tools/sim-linux-docker/demo.md` (Vietnamese): the flags, the obstacle option, the benchmark command, the overtaking behaviour and the alerts.

## Steps

1. Run the full matrix with the chosen flags, 10 repeats.
2. Run it once more with all flags off and confirm it equals the baseline.
3. Write the tables and the list of rows that fail, each with its reason and follow-up.
4. Update `demo.md`.

## Validation

Every acceptance row of `plan.md` is shown as pass, or as fail with the reason and the follow-up.

## Risk and rollback

Documentation and measurement only.
