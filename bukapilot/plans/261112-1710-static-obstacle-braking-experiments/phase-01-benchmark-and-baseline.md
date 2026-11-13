# Phase 1: Benchmark and baseline

## Context

Probes so far drove the ego with `set_target_velocity` and openpilot disengaged. They show what the model perceives, not whether the car stops. Candidate fixes must be judged on engaged runs with identical scenarios. `--lead-log` already prints ground-truth gap and collisions (`tools/sim/bridge/common.py`), but only for motorcycles and only as text.

## Requirements

- Spawn a world-fixed obstacle in the ego lane from the command line, at a given distance, of a given kind: motorcycle, car, car across the lane, street barrier, container.
- Ground-truth logging covers any obstacle, not only motorcycles.
- One machine-readable result per run: minimum gap, collision yes/no and impact speed, peak deceleration, time below -1.5 m/s², final speed, distance at which openpilot first had a lead.
- A runner that repeats a scenario N times headless and writes a summary table.
- An option to record every camera frame of a run with ground truth (gap, ego speed, pose), for the offline trials in phases 3 and 4.
- An "overtake from minimum gap" scenario: the car stops behind an obstacle, then a scripted steering and throttle input (the same commands the keyboard sends) moves it out and past. Recorded: whether the car braked against the input, the lateral offset openpilot reported for the obstacle (`yRel`) against ground truth, and contact.

## Files

- Edit `tools/sim/bridge/carla_world.py`: obstacle spawning from a spec string; `truth_lead()` considers all spawned obstacles and vehicles in lane.
- Edit `tools/sim/bridge/common.py`: per-run metrics collected from the existing lead-log path, written as one JSON file at exit (path from `LEAD_RESULT_FILE`).
- Edit `tools/sim-linux-docker/run.sh`, `start_docker.sh`: `--obstacle-spec "kind:distance_m[:speed_kph]"`, pass-through of `LEAD_RESULT_FILE`.
- Add `tools/sim-linux-docker/bench.sh`: loops scenarios and repeats, restarts the stack per run, collects JSON into a CSV.
- Edit `tools/sim/tests/test_sim_input.py`: tests for spec parsing and metric aggregation.
- Edit `tools/sim-linux-docker/demo.md` (Vietnamese): the new option and the benchmark command.

## Steps

1. Read how `--motorcycle-spec` is parsed and reuse that path for the obstacle spec.
2. Spawn obstacles with physics on, resting on the road. The Town05 check showed the placement method does not change the result on flat road; on Town04's slope use the waypoint height.
3. Add the metrics and the JSON output. Ground truth stays logging-only.
4. Write `bench.sh` with a short set (stationary motorcycle and clear road, 3 repeats, Town05, 30 km/h) and the full matrix from the plan.
5. Run the full matrix with current code and store it as the baseline in the report.
6. Find why the earlier headless run slid `vCruise` to 8 km/h; the benchmark needs a stable set speed.

## Validation

- `python3 -m unittest discover` on `tools/sim/tests` passes in a throwaway container.
- Two consecutive baseline runs of the same scenario give minimum gaps within 1 m of each other, or the spread is recorded as the benchmark's noise floor.
- With no obstacle spec and no `--lead-log`, bridge output is unchanged.

## Risk and rollback

Sim-side only. Rollback is reverting the bridge and script edits.
