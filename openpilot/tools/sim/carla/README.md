# openpilot + CARLA bridge

This integration runs openpilot in Ubuntu WSL2 and CARLA 0.9.16 natively on
Windows. It provides a simulated Tesla Model 3, CAN/panda messages, synchronized
camera frames, the openpilot UI, keyboard overrides, and automatic engagement
and recovery.

One road camera is enabled by default. Add `--dual_camera` only when a wide-road
camera is specifically needed.

Start with the [installation guide](installation.md). It contains the exact
fork URL, fresh-clone setup, Windows Firewall commands, Town10 and Town04 run
examples, longitudinal logging, and troubleshooting.

Typical launch from this directory:

```bash
bash run.sh all --carla-town Town10HD_Opt --carla-spawn-point 16
```

The CARLA launcher uses the large openpilot UI and fits its window to the
desktop monitor by default. Its camera view shows the complete sensor frame
(with side margins when the aspect ratios differ), while the camera sent to
the driving model is unchanged. For the old compact UI or a specific window
size, set `BIG=0` or `SCALE=...` before `bash run.sh`; use
`CARLA_MONITOR_FULL_FRAME=0` to restore the standard cropped driving view.

Optional openpilot longitudinal control:

```bash
bash run.sh all --openpilot-longitudinal \
  --carla-town Town10HD_Opt --carla-spawn-point 16
```

## Vietnam dense motorcycle weave

The scene commands 30 km/h cruise and starts after openpilot longitudinal is
active and ego reaches 7.5 m/s. It maintains 3–5 Vespas in the two adjacent
lanes at 5–7 m/s. A warmed-up motorcycle from that flow becomes the next
cut-in only when its measured speed and position provide a suitable gap;
another bike replenishes the side lane. Seeded attempts are due every 2–3 s,
with a 4.5–5.5 m/s cut-in target and roughly 6–8 m measured entry gap. The
physical lane hold is measured from the bike's actual position, not assumed
from the schedule. A new attempt waits while the previous crossing is active.
The scene checks vehicle footprints at spawn and defers on occupied lanes or
insufficient road; it does not guarantee collision avoidance after a close
cut-in. Actors are retired after crossing or leaving the available road; none
are teleported in front of ego, and their velocity is initialized only once
at spawn rather than forced every tick.

```bash
bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-case alternating \
  --carla-scene-seed 42 --carla-scene-duration 0 --no-experimental-mode
```

`--carla-scene-duration 0` disables the scene timer; stop manually with Ctrl-C
or `bash run.sh stop`. Positive durations remain available for finite test runs
(default 45 s). This does **not** reset the ego or loop its route: Town04's
straight highway eventually ends, so indefinite traffic generation does not
guarantee the ego can drive forever.

Use `--experimental-mode` for the comparison run. A named scene defaults to
experimental mode off when neither flag is supplied, so the baseline cannot
inherit stale Params from a previous simulator session.

The bridge writes timestamped `events.jsonl` segments below `.carla/reports/`,
rotating at 10 MiB. Event records are immediate; full ground truth is sampled
at 2 Hz and compact control/lead measurements at 10 Hz. The analyzer reports
cut-ins with lead or real brake actuation, stop/go, clearance, and collisions.
`manifest.json` records source/model/runtime hashes and run settings; its
`compiled_source_link_verified` flag remains false until the compiled model
is independently tied to the checked ONNX source. Lead–actor matching is
offline only and reports unknown when geometry is ambiguous:

```bash
uv run python ../analyze_motorcycle_weave.py \
  ../../../../.carla/reports/motorcycle-weave-YYYYMMDD-HHMMSS-NNNNNNNNN
```

### Phase 2: simulation-only Vietnam traffic mode

The [Phase 2 architecture note](../../../../docs/architecture/vietnam-traffic-phase2.md)
maps the planner, CARLA bridge, telemetry, and the remaining acceptance gates.

`VN_TRAFFIC_MODE=1` enables an opt-in low-speed positive-acceleration ramp in
the longitudinal planner. It is active only with `SIMULATION=1`, openpilot
longitudinal engaged, and experimental mode off. The default is `0` (stock).
`VN_TRAFFIC_PROFILE` accepts `balanced` (default), `gentle`, or `responsive`.
The policy leaves deceleration/braking requests, lead selection, following
distance, model weights, and actuator bridge unchanged. It must not be treated
as a vehicle-ready safety improvement.

Run a 60-second visible baseline in WSL, then repeat with the candidate; use
the same town, spawn, seed, camera, Params, and scene settings for each pair:

```bash
VN_TRAFFIC_MODE=0 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-case alternating \
  --carla-scene-seed 42 --carla-scene-duration 60 --no-experimental-mode

VN_TRAFFIC_MODE=1 VN_TRAFFIC_PROFILE=balanced bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-case alternating \
  --carla-scene-seed 42 --carla-scene-duration 60 --no-experimental-mode
```

Use `gentle` and `responsive` for the other tuning runs; seeds 42–44 are for
selection and 45–49 for independent confirmation. Run episodes sequentially,
not headless. After each pair, compare the two report directories in order:

```bash
uv run python ../analyze_vn_traffic.py \
  ../../../../.carla/reports/BASELINE_DIRECTORY \
  ../../../../.carla/reports/CANDIDATE_DIRECTORY
```

The report streams rotated JSONL segments and shows actual/requested jerk,
distance, observed lead TTC, contact and disengagement. It marks irregular
10 Hz telemetry invalid and excludes comfort samples after the first contact.
A passing single pair is not Phase 2 acceptance; the full validation set and
visual clips are still required.

### Trustworthy Phase 1 / Phase 2 comparison gates

Phase 1 is stock Openpilot (`VN_TRAFFIC_MODE=0`); Phase 2 is the existing
simulation-only policy (`=1`). The scene and observer code are shared. Each
visible, finite run writes a manifest, rotated 10 Hz control/actor records,
2 Hz ground truth, and a separate rotated planner-input trace. Replay the
*same* recorded inputs through the stock Phase 1 planner and the current
mode-0 planner before scoring a run:

```bash
.venv/bin/python -m openpilot.tools.sim.planner_trace \
  .carla/reports/REPORT_DIRECTORY
.venv/bin/python -m openpilot.tools.sim.analyze_vn_traffic \
  .carla/reports/BASELINE_DIRECTORY .carla/reports/CANDIDATE_DIRECTORY
```

Run these commands from the repository root in WSL. The comparison prints
`run_valid` for each run, then separate `comparison_valid` and
`phase2_improved`; `pass` requires both. A missing or ambiguous radar lead is
reported as an observation, not silently assigned to the cut-in motorcycle.
The scene records actual cut-in exposures (entry gap, ego/bike speed, lane
hold, contact, and evidence state). It compares only same-direction exposures
within fixed calipers: 2 m entry gap, 1.5 m/s ego speed, and 0.5 s lane hold;
each pair needs at least two matches per direction. Scheduled 6–8 m gaps and
1.5–2 s holds remain targets and are reported rather than post-hoc gates.
Lateral P95/max remains a diagnostic, never a criterion used to select runs.
Control, ground truth, pose attribution, and planner traces must cover the
entire 60 s episode without sample gaps; an Openpilot snapshot older than
250 ms invalidates attribution. Contact is retained as an outcome and comfort
after first contact is excluded.

The manifest hashes the model sources, compiled artifacts, and relevant
source files, and records dirty source paths. `model-runtime.json` records
the artifact chunks actually loaded by modeld. The ONNX-to-compiled-artifact
link is a separate required gate: a runtime hash match alone does **not**
establish that link. Until a reproducible build proves it, reports must say
`run_valid=false` and must not claim Phase 2 improvement. Existing older
reports are diagnostic only, not acceptance evidence. After scene tracking
passes on seeds 42–44, freeze the scene and run seeds 45–49 twice per seed
and mode, sequentially with the same town/spawn/camera/Params/model/bridge.
Have a person capture a timestamped clip for each cut-in direction and
compare it with lead, acceleration, and actual brake logs.
