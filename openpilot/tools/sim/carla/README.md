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

Branch names do not select behavior. `VN_TRAFFIC_MODE=0` is Original stock
Openpilot and `VN_TRAFFIC_MODE=1` enables the simulation-only enhancement;
both modes run the same named scene variants.

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

### Original close cut-in baseline

`close_cut_in` is a scene-only stress case for the Original baseline. One
motorcycle enters the ego lane at a measured target gap of 2.5–3.5 m while
the ego is traveling at 30 km/h, then remains in lane for 1.5–2.0 seconds.
It is intentionally close enough that contact is an expected result to
measure; it does not change model weights, radar, planner, MPC, policy, or
the actuator bridge. The actor is first staged in an adjacent lane, never
spawned overlapping the ego, and never teleported in front of it.

Run this visibly in WSL from this directory. It schedules right→left first,
then left→right in the same episode. If the first cut-in contacts the ego, the
second is recorded as deferred rather than forced into compromised physics.
Use the same command with `VN_TRAFFIC_MODE=1` only later when comparing the
enhancement; that comparison is outside this Original scene implementation.

```bash
VN_TRAFFIC_MODE=0 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case alternating --carla-scene-seed 42 \
  --carla-scene-duration 30 --no-experimental-mode
```

For the acceptance set, run seeds 42, 43, and 44 and keep every report,
including runs that avoid contact or defer the second direction. Analyze each
exact report directory with the command above; the summary reports the entered
gap, entry TTC, active-cut-in contacts, and other contacts.

### Phase 2: simulation-only Vietnam traffic mode

The [Phase 2 architecture note](../../../../docs/architecture/vietnam-traffic-phase2.md)
maps the planner, CARLA bridge, telemetry, and the remaining acceptance gates.

#### Optional CARLA radar-assisted cut-in (experimental)

`VN_RADAR_CUTIN=1` adds a forward CARLA radar sensor and a simulation-only
tracker. The sensor publishes only range/angle/height/radial-velocity returns;
actor IDs, actor poses, and the scene schedule remain evaluation-only. Phase 2
combines a confirmed radar/path conflict with the existing model cut-in obstacle
in MPC. `VN_TRAFFIC_MODE=0` still uses the stock planner/MPC path, even when
the sensor is enabled for a paired comparison. The model and compiled artifact
are not rebuilt, and the bridge is not told to brake.

Run both modes visibly and sequentially from WSL in this directory. The
`alternating` case tests right-to-left and left-to-right within one episode:

```bash
VN_RADAR_CUTIN=1 VN_TRAFFIC_MODE=0 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case alternating --carla-scene-seed 42 \
  --carla-scene-duration 30 --no-experimental-mode

VN_RADAR_CUTIN=1 VN_TRAFFIC_MODE=1 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case alternating --carla-scene-seed 42 \
  --carla-scene-duration 30 --no-experimental-mode

bash run.sh stop
```

From the repository root, the finite-run diagnostic below replays measured
radar returns against the planner path and matches alerts to scene actors
**only after planning**, for evaluation. It reads all rotated radar/planner
files from the report directory:

```bash
.venv/bin/python -m openpilot.tools.sim.analyze_radar_cutin \
  .carla/reports/REPORT_DIRECTORY
```

Early visible tests showed braking and two completed cut-ins without contact,
but the motorcycle lateral tracking gate remains invalid and no synchronized
video was available. This is preliminary simulation evidence, not an accepted
collision-avoidance result or a real-vehicle feature. See the
[radar cut-in report](../../../../docs/reports/2026-09-24-vn-radar-cutin-preliminary.md).

`VN_TRAFFIC_MODE=1` enables three opt-in low-speed policies in the longitudinal
planner: model-based early cut-in prediction, a positive-acceleration ramp, and up to 0.30 s extra following time
when a fresh `leadOne` or `leadTwo` is closing with TTC below 5 s. It is active
only with `SIMULATION=1`, openpilot longitudinal engaged, and experimental
mode off. The default is `0` (stock). `VN_TRAFFIC_PROFILE` accepts `balanced`
(default), `gentle`, or `responsive`. Model weights, compiled model and the
actuator bridge are unchanged. The cut-in predictor uses modelV2 lead
hypotheses and ego path only; it does not receive CARLA actor positions or
force brake actuation. It adds a future obstacle to the existing MPC only
after three fresh confirming frames. It can confirm either a model trajectory
crossing or consistent frame-to-frame lateral motion, and keeps a confirmed
track through short uncertainty using a 0.3 s retraction. The positive
acceleration ramp also starts at standstill, avoiding a full first request
followed by a sharp reduction as the car begins moving. In the latest visible
seed-42 run, throttle changed smoothly after engagement instead of alternating
between full throttle and brake. The run still contacted the motorcycle, and
its live model observations did not confirm a cut-in. A replay of an earlier
trace confirmed 0.316 s before lane entry but did not make MPC choose `lead2`.
This is **not** an accepted collision-avoidance improvement or a vehicle-ready
safety feature.

Run the same close-cut-in scene in Original and Enhancement modes, visibly
and sequentially in WSL. Use `right_to_left`, `left_to_right`, or `alternating`
as the case; the one-direction cases remain useful when a first contact
defers the second crossing:

```bash
VN_TRAFFIC_MODE=0 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case right_to_left --carla-scene-seed 42 \
  --carla-scene-duration 30 --no-experimental-mode

VN_TRAFFIC_MODE=1 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case right_to_left --carla-scene-seed 42 \
  --carla-scene-duration 30 --no-experimental-mode
```

Controls share the same 30 km/h ego and 30-second report but schedule no
cut-ins: `--carla-scene-case empty_road` has no traffic,
`adjacent_only` keeps motorcycles in adjacent lanes, and `steady_lead`
spawns one same-lane car at 35 m targeting 7 m/s. These controls do not
alter the `close_cut_in` case. From the repository root, inspect each run:

```bash
.venv/bin/python -m openpilot.tools.sim.analyze_motorcycle_weave \
  .carla/reports/REPORT_DIRECTORY
.venv/bin/python -m openpilot.tools.sim.planner_trace \
  .carla/reports/ORIGINAL_REPORT_DIRECTORY \
  --stock-ref 6a125380ac8e3b261ed1feb60a9eb8d6d84e18a5
```

The summary lists each cut-in separately, including actual entry gap,
contact, confirmation and MPC/deceleration timing. A matching side is only
*side-compatible*, not proof that a model hypothesis identified that actor.
`lead2` marks the lowest MPC obstacle margin, not proof of braking causality.
Deferred crossings and missing model evidence cannot count as successes.

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
the artifact chunks actually loaded by modeld. A relative Mode 0/Mode 1
comparison requires each receipt to match its manifest and the exact loaded
chunk hash map to match between runs. The ONNX-to-compiled-artifact link is
reported separately as `compiled_source_link_verified`; the working compiled
model is not rebuilt for this comparison. Existing older reports are
diagnostic only, not acceptance evidence. After scene tracking
passes on seeds 42–44, freeze the scene and run seeds 45–49 twice per seed
and mode, sequentially with the same town/spawn/camera/Params/model/bridge.
Have a person capture a timestamped clip for each cut-in direction and
compare it with lead, acceleration, and actual brake logs.
