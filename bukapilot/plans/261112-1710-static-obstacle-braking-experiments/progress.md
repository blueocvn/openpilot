# Progress: static obstacle braking experiments

Last update: 2026-11-13 12:45. Work paused before the full baseline run (phase 1, step 5); the user will run it.

## State

| Phase | State | Note |
|---|---|---|
| 1. Benchmark and baseline | Code done, every scenario exercised once; full baseline not run | `BENCH_JOBS=2 bash bench.sh full`, about 45 to 60 minutes |
| 2. Hypothesis checks | H2 done (rejected); H1, H3, H4 not started | H1 and H4 need the phase 1 runner in Town04 |
| 3 to 6 | Not started | Phases 3 and 5 need approval before any `selfdrive/` edit |

Nothing is committed. `selfdrive/` is untouched. The benchmark stack (`carla_bench`, `bukapilot_bench`) is stopped.

## What was built (phase 1)

- `tools/sim/bridge/run_metrics.py` (new): obstacle spec parser, per-run scoring, scripted overtake driver, frame recorder.
- `tools/sim/bridge/bench_summary.py` (new): pass rules of the acceptance table, table and CSV.
- `tools/sim-linux-docker/bench.sh` (new): `short`, `full`, `run`, `summary`, `stop`. Own CARLA on port 2100, own bridge container.
- `tools/sim/bridge/carla_world.py`: world-fixed obstacles, ground truth for every obstacle, ego pose.
- `tools/sim/bridge/common.py`: scoring and recording hooks, scripted driver, `--cruise-kph` can now lower the set speed.
- `tools/sim/run_bridge.py`: `--obstacle-spec`. `start_docker.sh`: passes the new environment variables.
- `tools/sim/tests/test_sim_input.py`: 61 tests pass in a throwaway container.
- `tools/sim-linux-docker/demo.md`: the option, the variables and a benchmark section.

## Decisions (user, 2026-11-13)

1. Clear road keeps its original spawn points and 2 km length (Town04 170, Town05 1, with bends).
2. The motorcycle drives straight and the follow run is longer.
3. Obstacles are placed farther away. Measured limit: Town05 has no straight road longer than 360 m, so Town05 uses 250 m; Town04 spawn 170 is straight for 450 m and uses 400 m.
4. Intersections are out of scope here: the user will handle them as a separate case later. Town05 60 km/h is skipped for that reason.
5. One map only, Town04_Opt (user, 2026-11-13); Town05 dropped from the benchmark. The matrix is 14 cells, 3 repeats.
6. The 4 m minimum gap was explained to the user, no change requested; it stays in the plan.

## Findings so far

The Town05 findings below are historical: Town05 is no longer benchmarked.


1. Set speed (phase 1, step 6). `--cruise-kph` only pressed RES+, so a 30 km/h run stayed at the 40 km/h engage default. Fixed in the bridge; later runs hold 30. The earlier slide to 8 km/h was not reproduced and is still unexplained.
2. Road geometry, measured on the map. Longest straight road from any spawn point: Town05 360 m (spawns 88, 126, 127; an intersection every 90 m, 25 m long), Town04 715 m (spawns 171 to 173, which climb 11 m). Longest stretch without any junction is 155 m in either town. Spawn points used: Town04 170 (straight 450 m, junction at 245 to 300 m), Town04 171 for following, Town05 126.
3. Motorcycle that moves: CARLA 0.9.12 has no Traffic Manager route call, so the motorcycle would turn at junctions. A driven motorcycle (`motorcycle:distance:kph` in `--obstacle-spec`) now follows the straightest road at constant speed, moved by the bridge each step with physics off. Hitting it is a hard contact.
4. Town05 at 60 km/h is not usable: in 4 of 4 runs the car left the straight inside the first intersections (no lane lines), drove 390 m and hit a fence without meeting the obstacle. Town05 runs only at 30 km/h; the full matrix skips Town05 at 60 km/h. A car crawling at 2 m/s inside an intersection also lost its lane (hit a traffic light), so the slow-motorcycle run ends at 160 m (Town05) and 230 m (Town04), before the next junction.
5. H2 rejected. The bridge's BGRA to I420 conversion is BT.601 limited range: luma is 1.07 levels above the reference on average (maximum 2.6), chroma within 1 level. It is the same arithmetic as `selfdrive/camerad/transforms/rgb_to_yuv.cl`.
6. Scoring uses the simulated clock (`realtime_factor` is about 1.0 to 1.1), and braking figures stop at the first contact.

7. Speed of the benchmark (2026-11-13). The simulation cannot run faster than real time without changing what openpilot perceives: `modeld` and `controlsd` use the wall clock. The first `full` run was slow because four of five Town05 runs never met the obstacle (it stood past the intersection at 170 m, where the car leaves the straight) and ran 146 s each, with 10 repeats per cell. Changes: Town05 obstacle at 150 m, 3 repeats per cell (runs agree within about 1 m), per-run time limit from distance and speed, clear road 1000 m, and `BENCH_JOBS=n` stacks side by side (own CARLA port 2100 + 10 n and own containers). Measured with the user's sim also running: 2 stacks keep `realtime_factor` at 1.0 to 1.2, 3 stacks drop to 0.8, so use 2. The summary fails any run below 0.95.

## Phase 3 so far (2026-11-13)

Code (needs the user's approval before any default changes; all three flags are off by default): `VisionLeadFilter` in `selfdrive/controls/lib/radar_helpers.py`, hooked into `get_lead` in `selfdrive/controls/radard.py`, three Params keys in `selfdrive/common/params.cc`, 17 unit tests in `selfdrive/controls/tests/test_vision_lead.py`. Parts: A distance margin, B speed from distance trend, C lead hold.

Offline replay of the baseline traces rejected B (10% of following frames had a speed more than 2 m/s too low, so phantom braking; the model's speed error is already small) and kept A and C.

Benchmark contamination found: the CARLA server of the benchmark outlives the runs and a run can end without destroying its actors, so the ego car of earlier runs stood on the road (collisions with `vehicle.tesla.model3` on an empty road, always at about 385 m). Every Town04 result before this fix is unreliable, including the earlier baseline and its apparent noise. Fix: `CARLA_CLEAN_START=1` (set by `bench.sh`) removes leftover vehicles, props and sensors at the start of each run.

Follow test, ego 60 km/h, motorcycle 40 km/h, 3 runs per setting, 1 stack, after the contamination fix (`.carla/bench/flags-follow/`). Measured on the frames where both cars drive within 1 m/s of each other (the real steady follow), true gap against the gap the planner aims for (`desired_follow_distance`):

| Setting | Closest approach per run (m) | True gap / aimed gap, per run | Mean | Runs within 20% of the aimed gap |
|---|---|---|---|---|
| base | 2.65, 4.54, 4.82 | 0.56, 0.76, 0.62 | 0.65 | 0 of 3 |
| A margin | 6.69, 6.20, 6.30 | 0.75, 0.63, 0.88 | 0.75 | 1 of 3 |
| C hold | 5.13, 5.49, 5.09 | 0.84, 0.60, 0.65 | 0.69 | 1 of 3 |
| A + C | 8.29, 6.15, 6.60 | 0.93, 0.85, 0.95 | 0.91 | 3 of 3 |

The gap oscillates by about 5 m (standard deviation) in every setting. No collisions, no false braking, peak deceleration unchanged (-1.1 to -2.1 m/s²). Flag A confirmed active from the traces (radard's dRel is 3.0 m below the model's, the minimum margin); C holds the lead for 193 frames (A+C: 336) against 15 in base. Only A+C separates clearly from base; A and C alone overlap base. Limits: 3 runs per setting; the stop-in-front-of-obstacle cells were not run for A and C; the bench summary's own "steady gap" metric returned nothing in 9 of 12 runs (its 10 s window falls in the bend after 715 m, where the motorcycle leaves the lane by ground truth), so its pass counts (base 2, A 1, C 2, A+C 0 of 3) are not meaningful and the metric must be changed to the matched-speed frames used above.

## Baseline numbers so far (stock code, flags do not exist yet)

Results are in `.carla/bench/` (git-ignored, on this machine only).

| Scenario | Town, set speed | Result |
|---|---|---|
| Stationary motorcycle, 3 runs | Town05, 30 km/h, obstacle at 150 m | 3 of 3 collisions at 5.7 to 5.8 m/s; first lead at 15 to 17 m true gap (`pilot-baseline`) |
| Stationary motorcycle, 3 runs | Town05 spawn 126, 30 km/h, obstacle at 150 m, 2 stacks side by side | 3 of 3 collisions at 1.4, 2.2 and 2.3 m/s; first lead at 54 m median; 28 s per run (`parallel-2`) |
| Stationary motorcycle, 3 runs | Town05, 30 km/h, obstacle at 250 m on spawn 1 | 3 of 3 collisions at 2.9 to 4.4 m/s; first lead at 31 to 33 m (`short-baseline-1`, development run) |
| Slow motorcycle (2 m/s), 1 run | Town05, 30 km/h | Pass: minimum gap 4.08 m, peak deceleration -2.2 m/s², first lead at 50 m (`new-scenarios3`) |
| Follow motorcycle at 40 km/h, 1 run | Town04 spawn 171, 60 km/h | Pass: steady gap 19.6 m against 22.8 m intended (14% off), 1301 m driven in 105 s, minimum gap 3.2 m (`new-scenarios3`) |
| Clear road 600 m, 3 runs | Town05 spawn 1, 30 km/h | 2 clean at 8.3 m/s average; 1 left the road in a bend (`short-baseline-1`) |

Not yet run with the final layout: Town04 obstacle scenarios (400 m), the overtake scenario, the frame recorder (`LEAD_RECORD_DIR`), parked car, barrier and container.

## To continue

1. Rerun the Town04 60 km/h cells (the 3-stack test there was overloaded and is invalid; its two collisions at 16 m/s, with no lead seen before contact, need confirming), then the full matrix:

   ```bash
   cd tools/sim-linux-docker
   bash bench.sh run stationary-motorcycle Town04_Opt 30 3    # noise floor in the other town
   BENCH_JOBS=2 bash bench.sh full                             # 3 repeats per cell, 21 cells
   bash bench.sh stop
   ```

2. Write the baseline table and findings into `plans/Report/report.md`.
3. Phase 2: H1 and H4 with `bench.sh run stationary-motorcycle Town04_Opt 30`, H3 only if a real-drive log is available.
4. Phases 3 and 5 change `selfdrive/` and need the user's approval before any edit.

## Open questions

1. Overtake scenario: the scripted driver (turn left with throttle, straighten, pass) has not been run yet and its timings are a first guess.
None open. Intersections are a separate case that the user will handle later (decision 2026-11-13), so Town05 at 60 km/h stays out of this plan's matrix and no other spawn point is needed.
