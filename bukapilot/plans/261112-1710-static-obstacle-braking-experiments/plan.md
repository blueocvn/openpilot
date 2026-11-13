---
title: Static obstacle handling and close-range braking experiments
status: draft, awaiting approval
mode: auto (standard)
created: 2026-11-12
revised: 2026-11-12 (assured clear distance, overtaking rules, simulation-only scope)
platform: tools/sim-linux-docker (Linux Docker, CARLA 0.9.12, camera-only)
evidence: ../Report/report.md
blockedBy: []
blocks: []
---

# Static obstacle handling and close-range braking experiments

## Outcome

1. A repeatable benchmark that scores every candidate fix with the same scenarios and the same numbers (minimum gap, collisions, peak braking, false braking).
2. Measured answers to the open hypotheses in [the report](../Report/report.md).
3. The planner stops treating "no lead" as "road is clear". Speed is limited by the distance that is confirmed clear ahead (`d_clear`), so the car slows down smoothly as that distance shrinks and brakes to a stop before contact. The driver is warned and steers around the obstacle.

## Core idea: assured clear distance ahead

Today the planner has two states: a lead, or an empty road. A container that the model does not recognise falls into "empty road". The plan separates three states:

| State | Meaning | Speed |
|---|---|---|
| Confirmed clear to `d_clear` | Positive evidence that the lane is free up to that distance | Up to `v_max(d_clear)`, never above set speed |
| Lead | An object at a known, conservative distance | Existing MPC following, with the conservative distance |
| Unknown | Nothing detected, nothing confirmed | `d_clear` falls back to the worst measured seeing distance (25 m) |

`v_max = -a * t_r + sqrt((a * t_r)² + 2 * a * (d_clear - d_stop))`, with `a` the allowed braking (`COMFORT_BRAKE = 2.0`), `t_r` the system delay (measured 0.35 s) and `d_stop` the standstill gap.

`d_clear` is the minimum of the conservative lead distance (phase 3) and the free distance reported by an independent, class-agnostic source (phase 4). The planner cannot repair perception: without that source, `d_clear` stays at the fallback and the car is capped near 29 km/h.

## Constraints

- Camera only. CARLA ground truth is used for scoring and logging, never as controller input.
- The scope is the simulator; there is no real vehicle for now. The code still must not contain `SIMULATION`-gated behaviour, so that it stays valid if a real car is added.
- New mechanisms sit behind flags, default off, until the user decides after phase 6.
- Every `selfdrive/` change needs the user's approval before it is written (phases 3 and 5, and phase 4 only if its trial passes).
- With the flags off, longitudinal behaviour is unchanged.
- Sim tests run in a throwaway container, not inside the live `bukapilot_sim_carla`.

## Non-goals

- Retraining or replacing the driving model.
- Radar, lidar or any sensor other than the road camera.
- Automatic steering around the obstacle. The driver steers.
- Lane-change logic, lateral planner changes.
- Driving through intersections (no lane lines): a separate case, handled later. Benchmark runs are placed on roads where the obstacle is not inside or right after an intersection.

## Acceptance criteria

Each scenario runs 3 times (runs agree within about 1 m), engaged, at set speeds 30 and 60 km/h, in Town04_Opt only (spawn 170; spawn 171 for following). Town05 was dropped: its intersections are a separate case.

| Scenario | Pass condition |
|---|---|
| Stationary motorcycle in lane | 0 collisions, minimum gap at least 4 m |
| Motorcycle at 2 m/s in lane | 0 collisions, minimum gap at least 4 m |
| Parked car, street barrier | 0 collisions, minimum gap at least 4 m |
| Small container (never becomes a lead today) | 0 collisions, minimum gap at least 4 m |
| Following a motorcycle at 40 km/h | Steady gap within 20% of the planner's intended gap |
| Clear road, 2 km | 0 false braking events (deceleration below -1.5 m/s² with no object within 60 m), and average speed at least 90% of set speed |
| Overtake from minimum gap | Stopped 4 m behind an obstacle, the driver steers out and passes without the car braking against it and without contact |

In every run peak deceleration stays at or above -3.5 m/s² unless a collision would otherwise occur. Flags off: results equal the phase 1 baseline.

## Phases

| # | Phase | Main files | Depends on | Touches `selfdrive/` |
|---|---|---|---|---|
| 1 | [Benchmark and baseline](phase-01-benchmark-and-baseline.md) | `tools/sim/bridge/`, `tools/sim-linux-docker/run.sh`, `tools/sim/tests/` | none | no |
| 2 | [Hypothesis checks](phase-02-hypothesis-checks.md) | probes only, report | 1 | no |
| 3 | [Conservative lead](phase-03-conservative-lead.md) | `selfdrive/controls/radard.py`, `lib/radar_helpers.py` | 1, 2 | yes |
| 4 | [Clear-distance source trial](phase-04-clear-distance-source-trial.md) | offline scripts, then a new process if the trial passes | 1 | only after the trial passes and the user approves |
| 5 | [Clear-distance speed limit and close-range braking](phase-05-clear-distance-speed-limit-and-braking.md) | `selfdrive/controls/lib/longitudinal_planner.py`, `controlsd.py` | 3, 4 | yes |
| 6 | [Final evaluation and docs](phase-06-final-evaluation.md) | report, `demo.md` | 5 | no |

Phases 3 and 4 can run side by side after phase 1.

## Decision gates

- After phase 2: if the camera conversion is wrong, fix it and redo the baseline before anything else.
- After phase 3: keep only the parts that improve the benchmark without false braking.
- After the phase 4 trial: choose the clear-distance source, or record that none passed. If none passed, phase 5 ships with the 25 m fallback only and the container row is marked as not met.
- After phase 5: the user chooses flag defaults.

## Evidence this plan rests on

- Seeing distance depends on the object class: parked car beyond 100 m, motorcycle 30 to 45 m, street barrier 25 m, small container never. A fixed cap for all situations would treat the wrong problem.
- Distance is over-estimated 5 to 7 m at close range in both towns; the planner stops at 6.4 m (`long_mpc.py:59`).
- Vision leads pass `x` and `v` to the planner unchecked (`radar_helpers.py:133`), and only when `prob > 0.5` (`radard.py:71`). With no lead the MPC invents a fast lead 50 m ahead (`long_mpc.py:323-327`).
- `leadsV3.xStd` is 1 to 4 m when `x` is right and 12 to 60 m when it is wrong.
- The model's path, `hardBrakePredicted` and brake probabilities do not react to an obstacle, so they cannot provide `d_clear`.
- Camera geometry is verified within 2 px at 6 to 10 m, which the ground-plane candidate in phase 4 relies on.
- Town04 only: a stationary motorcycle is reported at about 7 m/s. Town05: about 2 m/s. Cause unknown.

## Risks

- No candidate in phase 4 may give a clean `d_clear`. Then the container case stays unsolved and only the fallback cap protects against it.
- A noisy `d_clear` makes the car surge and brake. The clear-road row (speed at least 90% of set speed, 0 false braking) measures this.
- Conservative distance trades collisions for false braking. Phase 3 measures the noise offline before any code change.
- Benchmark runs are slow (about 1 minute each). The full matrix is about 5 hours per candidate; phase 1 defines a 20 minute short set for iteration.

## Open questions

1. Minimum gap: 4 m is proposed. Confirm or change.
2. Is a real-drive log available? It is optional now (phase 2, H3).
