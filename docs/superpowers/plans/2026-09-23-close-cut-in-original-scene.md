# Close Cut-In Original Scene Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deterministic, single-event CARLA motorcycle cut-in variant that produces a close, physically generated hazard for Original Openpilot and records enough evidence to reuse the identical scene for a later enhancement comparison.

**Architecture:** Add `scene_variant` as an orthogonal input beside the existing direction `case`, then pass it from the CLI through `CarlaBridge` and `CarlaWorld` into `MotorcycleWeaveScene`. A variant configuration drives the existing seeded scheduler and moving-background promotion flow; telemetry and the streaming analyzer identify contact with the active cut-in even when the event cannot finish after impact.

**Tech Stack:** Python 3, CARLA 0.9.16, openpilot simulator bridge, `unittest`, Ruff, JSONL rotating reports, WSL Ubuntu.

**Spec:** `docs/superpowers/specs/2026-09-23-close-cut-in-original-scene-design.md`

## Global Constraints

- Original is `VN_TRAFFIC_MODE=0`; Enhancement is `VN_TRAFFIC_MODE=1`; branch names do not define behavior.
- This plan changes scene/configuration/telemetry/analyzer/docs only. Do not modify `openpilot/selfdrive/modeld`, `radard`, planner, MPC, `vn_traffic_policy.py`, longitudinal bridge control, model weights, or compiled model artifacts.
- Both modes must consume the same `close_cut_in` scene implementation and seed protocol.
- Run CARLA visibly in WSL. Do not add a headless acceptance path.
- Ego cruise remains exactly `30 km/h`; scene readiness remains `longActive` with `vEgo >= 7.5 m/s`.
- The motorcycle must already be moving in an adjacent lane. Do not spawn overlapping actors, teleport an actor in front of ego, or force velocity each tick.
- `close_cut_in` creates at most one cut-in per episode and retains telemetry until the configured finite duration ends.
- Contact is an observed Original outcome. Scene code must never issue an ego brake command or alter an Openpilot signal to force it.
- Preserve the existing `standard` scene behavior and all existing CLI defaults.
- Preserve unrelated workspace changes, including the existing AGNOS deletion and untracked files.

## Review Focus

- An unknown scene variant must fail at argument parsing and direct scheduler construction; Task 1 adds both tests.
- A missing/stale background actor or insufficient road must defer the event without consuming seed, direction, or the one-event allowance; Task 2 adds this test.
- Actor promotion must retain the current transform and must never turn a spawn overlap into the expected collision; Task 2 extends the promotion test with close geometry.
- A collision with a background or unknown actor must not count as contact with the active cut-in; Task 3 adds role and analyzer tests.
- A collision can prevent `cut_in_finished`; the analyzer must still report the active cut-in contact and retain the incomplete exposure; Task 3 adds an end-of-stream collision test.

---

### Task 1: Add the scene variant interface and preserve the standard defaults

**Files:**
- Modify: `openpilot/tools/sim/run_bridge.py:28-79`
- Modify: `openpilot/tools/sim/bridge/carla/carla_bridge.py:28-62`
- Modify: `openpilot/tools/sim/bridge/carla/carla_world.py:37-106`
- Modify: `openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py:17-110`
- Test: `openpilot/tools/sim/tests/test_motorcycle_weave.py:214-240,644-656`

**Interfaces:**
- Consumes: existing `--carla-scene`, `--carla-scene-case`, seed, duration and report arguments.
- Produces: `--carla-scene-variant {standard,close_cut_in}`; `CarlaBridge(..., carla_scene_variant="standard")`; `CarlaWorld(..., scene_variant="standard")`; `MotorcycleWeaveScene(..., variant="standard")`; `ContinuousWeaveScheduler(case, seed, variant="standard")`.

- [ ] **Step 1: Write failing CLI and constructor tests**

Import `CARLA_SCENE_VARIANTS` from `run_bridge` and `SCENE_VARIANTS` from
`motorcycle_weave`, then add these cases to `TestMotorcycleWeaveSchedule`:

```python
def test_carla_cli_preserves_close_cut_in_variant(self):
  args = parse_args([
    "--simulator", "carla", "--carla-scene", "motorcycle_weave",
    "--carla-scene-case", "left_to_right",
    "--carla-scene-variant", "close_cut_in",
  ])
  self.assertEqual(args.carla_scene_variant, "close_cut_in")

def test_standard_variant_remains_the_default(self):
  args = parse_args(["--simulator", "carla", "--carla-scene", "motorcycle_weave"])
  self.assertEqual(args.carla_scene_variant, "standard")
  scheduler = ContinuousWeaveScheduler("alternating", seed=42)
  self.assertEqual(scheduler.variant, "standard")

def test_scheduler_rejects_unknown_variant(self):
  with self.assertRaisesRegex(ValueError, "unknown motorcycle weave variant"):
    ContinuousWeaveScheduler("alternating", seed=42, variant="unknown")

def test_cli_and_scheduler_expose_the_same_scene_variants(self):
  self.assertEqual(CARLA_SCENE_VARIANTS, tuple(SCENE_VARIANTS))
```

- [ ] **Step 2: Run the focused tests and confirm the new expectations fail**

Run from the repository root in WSL:

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_carla_cli_preserves_close_cut_in_variant \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_standard_variant_remains_the_default \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_scheduler_rejects_unknown_variant
```

Expected: FAIL because the CLI and scheduler do not yet accept `variant`.

- [ ] **Step 3: Define immutable variant configuration**

Add near the existing scheduler constants in `motorcycle_weave.py`:

```python
@dataclass(frozen=True)
class SceneVariantConfig:
  interval_s: tuple[float, float]
  speed_mps: tuple[float, float]
  merge_gap_m: tuple[float, float]
  entry_s: tuple[float, float]
  hold_s: tuple[float, float]
  exit_s: tuple[float, float]
  staging_margin_m: float
  max_events: int | None


SCENE_VARIANTS = {
  "standard": SceneVariantConfig(
    interval_s=(2.0, 3.0), speed_mps=(4.5, 5.5), merge_gap_m=(6.0, 8.0),
    entry_s=(1.15, 1.55), hold_s=(0.75, 1.05), exit_s=(1.3, 1.6),
    staging_margin_m=1.2, max_events=None,
  ),
  "close_cut_in": SceneVariantConfig(
    interval_s=(2.0, 2.5), speed_mps=(4.0, 4.8), merge_gap_m=(2.5, 3.5),
    entry_s=(1.0, 1.3), hold_s=(1.5, 2.0), exit_s=(1.3, 1.6),
    staging_margin_m=0.3, max_events=1,
  ),
}
```

Update the scheduler constructor without changing the default path:

```python
def __init__(self, case: str, seed: int, variant: str = "standard"):
  if variant not in SCENE_VARIANTS:
    raise ValueError(f"unknown motorcycle weave variant: {variant}")
  self.variant = variant
  self.config = SCENE_VARIANTS[variant]
```

- [ ] **Step 4: Pass the new argument through the bridge**

In `run_bridge.py`, add:

```python
CARLA_SCENE_VARIANTS = ("standard", "close_cut_in")

parser.add_argument("--carla-scene-variant", choices=CARLA_SCENE_VARIANTS, default="standard")
```

Importing CARLA code during generic argument parsing is undesirable, so define
the choices locally. The synchronization test from Step 1 fails if either list
changes without the other.

Add `carla_scene_variant` to `bridge_args`, add the defaulted parameter to
`CarlaBridge.__init__`, store it, pass it as `scene_variant` to `CarlaWorld`,
then pass it as `variant` to `MotorcycleWeaveScene`.

- [ ] **Step 5: Run the Task 1 tests and the existing CLI/default tests**

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_carla_cli_preserves_close_cut_in_variant \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_standard_variant_remains_the_default \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_scheduler_rejects_unknown_variant \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_cli_and_scheduler_expose_the_same_scene_variants \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_carla_cli_preserves_a_named_scene_and_seed \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_seeded_scheduler_repeats_directions_and_stays_inside_requested_ranges
```

Expected: PASS; the last test proves `standard` retained its current ranges.

- [ ] **Step 6: Commit the interface separately**

```bash
git add openpilot/tools/sim/run_bridge.py \
  openpilot/tools/sim/bridge/carla/carla_bridge.py \
  openpilot/tools/sim/bridge/carla/carla_world.py \
  openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py \
  openpilot/tools/sim/tests/test_motorcycle_weave.py
git commit -m "sim: add close cut-in scene variant"
```

### Task 2: Generate one close cut-in from a moving adjacent motorcycle

**Files:**
- Modify: `openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py:83-124,451-541,895-932`
- Test: `openpilot/tools/sim/tests/test_motorcycle_weave.py:214-451`

**Interfaces:**
- Consumes: `SceneVariantConfig`, `ContinuousPass`, existing background actor pool and `_start_pass(...)` promotion flow from Task 1.
- Produces: `ContinuousWeaveScheduler.exhausted: bool`; variant-driven `propose(...)`; `background_cut_in_ready(..., staging_margin_m: float)`; one committed close event per episode.

- [ ] **Step 1: Write failing tests for ranges, one-event exhaustion and deterministic replay**

```python
def test_close_cut_in_is_single_event_seeded_and_inside_hazard_ranges(self):
  first = ContinuousWeaveScheduler("right_to_left", seed=42, variant="close_cut_in")
  second = ContinuousWeaveScheduler("right_to_left", seed=42, variant="close_cut_in")
  event = first.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0,
                        actor_available=True)
  replay = second.propose(0.0, ego_speed_mps=8.33, available_road_m=100.0,
                          actor_available=True)
  self.assertEqual(event, replay)
  self.assertTrue(4.0 <= event.speed_mps <= 4.8)
  self.assertTrue(2.5 <= event.merge_gap_m <= 3.5)
  self.assertTrue(1.0 <= event.entry_s <= 1.3)
  self.assertTrue(1.5 <= event.hold_s <= 2.0)
  first.commit(event)
  self.assertTrue(first.exhausted)
  self.assertIsNone(first.propose(event.next_start_time_s, ego_speed_mps=8.33,
                                  available_road_m=100.0, actor_available=True))
```

- [ ] **Step 2: Write a failing deferral test that protects RNG and event allowance**

```python
def test_close_cut_in_deferral_does_not_consume_the_only_event(self):
  scheduler = ContinuousWeaveScheduler("left_to_right", seed=44, variant="close_cut_in")
  self.assertIsNone(scheduler.propose(0.0, ego_speed_mps=8.33,
                                      available_road_m=10.0, actor_available=True))
  self.assertFalse(scheduler.exhausted)
  event = scheduler.propose(0.0, ego_speed_mps=8.33,
                            available_road_m=100.0, actor_available=True)
  replay = ContinuousWeaveScheduler("left_to_right", seed=44, variant="close_cut_in").propose(
    0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True)
  self.assertEqual(event, replay)
```

- [ ] **Step 3: Run the new scheduler tests and confirm failure**

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_close_cut_in_is_single_event_seeded_and_inside_hazard_ranges \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_close_cut_in_deferral_does_not_consume_the_only_event
```

Expected: FAIL because the scheduler still uses hard-coded standard ranges and
has no exhaustion state.

- [ ] **Step 4: Drive scheduler sampling from `SceneVariantConfig`**

Replace each hard-coded `uniform(...)` range in `propose` with:

```python
low, high = self.config.interval_s
interval_s = self.rng.uniform(low, high)
low, high = self.config.speed_mps
speed_mps = self.rng.uniform(low, high)
low, high = self.config.merge_gap_m
merge_gap_m = self.rng.uniform(low, high)
low, high = self.config.entry_s
entry_s = self.rng.uniform(low, high)
low, high = self.config.hold_s
hold_s = self.rng.uniform(low, high)
low, high = self.config.exit_s
exit_s = self.rng.uniform(low, high)
```

Add:

```python
@property
def exhausted(self) -> bool:
  return self.config.max_events is not None and self.event_index >= self.config.max_events
```

Return `None` at the top of `propose` when exhausted. Do not increment
`event_index` until `commit(event)` succeeds.

- [ ] **Step 5: Make staging margin variant-specific**

Change the readiness interface and call site:

```python
def background_cut_in_ready(event, *, distance_m, ego_speed_mps, bike_speed_mps,
                            vehicle_length_buffer_m, staging_margin_m):
  required_distance = (event.merge_gap_m + staging_margin_m + vehicle_length_buffer_m +
                       (ego_speed_mps - bike_speed_mps) * event.entry_s)
```

Pass `self.scheduler.config.staging_margin_m` from `before_tick`. Keep the
existing actor speed tolerance, footprint checks and promotion without
`set_transform` or `set_target_velocity`.

- [ ] **Step 6: Stop scheduling after the committed close event**

Before looking for candidates in `before_tick`, add:

```python
if self.scheduler.exhausted:
  return
```

This return is after active actor updates and background maintenance, so
telemetry and actor cleanup continue. It only prevents another hazard.

- [ ] **Step 7: Extend the no-teleport promotion test for close geometry**

Run the existing promotion setup once per variant and retain its actor identity
and transform guard:

```python
for variant in ("standard", "close_cut_in"):
  scene = MotorcycleWeaveScene(
    SimpleNamespace(LaneType=SimpleNamespace(Driving="driving")),
    world, object(), [actor], variant=variant,
  )
  scene._background.append({"actor": actor, "side": "right", "speed_mps": 5.0})
  records = []
  scene._write = records.append
  event = scene.scheduler.propose(
    0.0, ego_speed_mps=8.33, available_road_m=100.0, actor_available=True,
  )
  self.assertTrue(scene._start_pass(event, Lane(2), Lane(3), Lane(1), scene._background[0]))
  scene.scheduler.commit(event)
  self.assertIs(scene._actors[0]["actor"], actor)
  self.assertFalse(records[0]["front_of_ego_teleport"])
  self.assertTrue(records[0]["spawn_footprints_clear"])
  self.assertEqual(scene.scheduler.exhausted, variant == "close_cut_in")
```

The mock actor's `set_transform` must continue raising `AssertionError`, which
proves promotion did not move the visible actor.

- [ ] **Step 8: Run the scheduler, promotion and standard-regression tests**

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_close_cut_in_is_single_event_seeded_and_inside_hazard_ranges \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_close_cut_in_deferral_does_not_consume_the_only_event \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_cut_in_promotes_a_moving_background_actor_without_teleport \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_spawn_uses_oriented_footprints_and_rejects_overlap \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_seeded_scheduler_repeats_directions_and_stays_inside_requested_ranges
```

Expected: PASS.

- [ ] **Step 9: Commit the close-event behavior**

```bash
git add openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py \
  openpilot/tools/sim/tests/test_motorcycle_weave.py
git commit -m "sim: generate one close motorcycle cut-in"
```

### Task 3: Record scene identity and count the correct collision

**Files:**
- Modify: `openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py:412-445,790-862,895-1061`
- Modify: `openpilot/tools/sim/analyze_motorcycle_weave.py:177-278`
- Test: `openpilot/tools/sim/tests/test_motorcycle_weave.py:151-208,474-525`
- Test: `openpilot/tools/sim/tests/test_vn_traffic_manifest.py:10-27`

**Interfaces:**
- Consumes: `MotorcycleWeaveScene.variant`, active actor list, existing collision callbacks and rotating JSONL stream.
- Produces: `scene_variant` in scenario/event/manifest records; collision `other_actor_role`; summary fields `scene_variant`, `cut_in_started_count`, `cut_in_entered_count`, `active_cut_in_contact_count`, `other_contact_count`, `entry_gap_m`, `entry_closing_speed_mps`, and `entry_ttc_s`.

- [ ] **Step 1: Write a failing manifest identity test**

Extend `test_manifest_records_opt_in_and_profile`:

```python
scene = MotorcycleWeaveScene(
  object(), object(), object(), [], variant="close_cut_in", report_dir=directory,
)
scene._write_manifest()
manifest = json.loads((scene.report_dir / "manifest.json").read_text())
self.assertEqual(manifest["scene_variant"], "close_cut_in")
```

- [ ] **Step 2: Write a failing collision-role test**

Import `deque` from `collections`, then add this test using simple actor/event
objects:

```python
def test_collision_callback_distinguishes_active_cut_in_background_and_other(self):
  active = SimpleNamespace(id=42, type_id="vehicle.vespa.zx125")
  background = SimpleNamespace(id=77, type_id="vehicle.vespa.zx125")
  other = SimpleNamespace(id=99, type_id="vehicle.tesla.model3")
  scene = MotorcycleWeaveScene.__new__(MotorcycleWeaveScene)
  scene.start_time_s = 10.0
  scene._actors = [{"actor": active}]
  scene._background = [{"actor": background}]
  scene._collisions = deque(maxlen=32)
  records = []
  scene._write = records.append

  for frame, actor in enumerate((active, background, other), start=1):
    scene._on_collision(SimpleNamespace(
      other_actor=actor, timestamp=11.0, frame=frame,
      normal_impulse=SimpleNamespace(x=1.0, y=0.0, z=0.0),
    ))

  self.assertEqual(
    [record["other_actor_role"] for record in records],
    ["active_cut_in", "background", "other"],
  )
```

- [ ] **Step 3: Write a failing streaming analyzer test for an unfinished collided event**

```python
def test_unfinished_close_cut_in_contact_is_counted_at_end_of_stream(self):
  records = [
    {"type": "scenario_started", "scene_time_s": 0.0,
     "scene_variant": "close_cut_in", "case": "right_to_left", "seed": 42},
    {"type": "ground_truth", "scene_time_s": 0.0, "ego_speed_mps": 8.3,
     "actors": [], "collisions": []},
    {"type": "cut_in_started", "scene_time_s": 5.0, "actor_id": 42},
    {"type": "cut_in_entered_ego_lane", "scene_time_s": 6.0, "actor_id": 42,
     "actual_gap_m": 3.0, "ego_speed_mps": 8.3, "bike_speed_mps": 4.3},
    {"type": "collision", "scene_time_s": 6.7, "other_actor_id": 42,
     "other_actor_role": "active_cut_in", "impulse_norm": 120.0},
    {"type": "ground_truth", "scene_time_s": 20.0, "ego_speed_mps": 0.0,
     "actors": [], "collisions": []},
  ]
  summary = summarize_records(iter(records))
  self.assertEqual(summary["scene_variant"], "close_cut_in")
  self.assertEqual(summary["cut_in_started_count"], 1)
  self.assertEqual(summary["active_cut_in_contact_count"], 1)
  self.assertEqual(summary["other_contact_count"], 0)
  self.assertAlmostEqual(summary["entry_ttc_s"], 0.75)
```

- [ ] **Step 4: Run the new telemetry tests and confirm failure**

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_vn_traffic_manifest \
  openpilot.tools.sim.tests.test_motorcycle_weave.TestMotorcycleWeaveSchedule.test_unfinished_close_cut_in_contact_is_counted_at_end_of_stream
```

Expected: FAIL because the new identity, role and summary fields do not exist.

- [ ] **Step 5: Add variant and measured entry data to records**

Add `scene_variant: self.variant` to `scenario_started`, `cut_in_started` and
`manifest.json`. Keep the existing scheduled `merge_gap_m`; the actual entry
record remains authoritative.

At lane entry calculate:

```python
closing_speed_mps = (ego_speed - bike_speed
                     if ego_speed is not None and bike_speed is not None else None)
entry_ttc_s = (gap_m / closing_speed_mps
               if gap_m is not None and closing_speed_mps is not None and closing_speed_mps > 0 else None)
```

Write `closing_speed_mps` and `entry_ttc_s` in `cut_in_entered_ego_lane`.

- [ ] **Step 6: Classify collision callbacks using current actor ownership**

Add a helper that only reads current scene state:

```python
def _actor_role(self, actor_id):
  if any(item["actor"].id == actor_id for item in self._actors):
    return "active_cut_in"
  if any(item["actor"].id == actor_id for item in self._background):
    return "background"
  return "other"
```

Store `other_actor_role` in the immediate collision record. Do not use this
classification in the planner or actor controller.

- [ ] **Step 7: Make the analyzer finalize contact independently of event completion**

Track constant-size counters while streaming:

```python
cut_in_started_count = 0
cut_in_entered_count = 0
active_cut_in_contact_ids = set()
other_contact_ids = set()
entry_gap_m = entry_closing_speed_mps = entry_ttc_s = None
```

On `cut_in_started`, increment the start count. On entry, retain the measured
fields for the single-event variant. On collision, add the actor ID to the
appropriate set immediately. Return set lengths in the summary. Do not wait for
`cut_in_finished` to count contact.

- [ ] **Step 8: Run telemetry/analyzer tests and all motorcycle scene tests**

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_motorcycle_weave \
  openpilot.tools.sim.tests.test_vn_traffic_manifest
```

Expected: PASS. The existing rotated-report streaming tests must remain green.

- [ ] **Step 9: Commit evidence changes**

```bash
git add openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py \
  openpilot/tools/sim/analyze_motorcycle_weave.py \
  openpilot/tools/sim/tests/test_motorcycle_weave.py \
  openpilot/tools/sim/tests/test_vn_traffic_manifest.py
git commit -m "sim: report close cut-in contact evidence"
```

### Task 4: Document and validate the Original close-cut-in scene

**Files:**
- Modify: `openpilot/tools/sim/carla/README.md:48-140`
- Create: `docs/reports/2026-09-23-close-cut-in-original-scene.md`
- Test: no product test file; run the focused suite, Ruff and visible CARLA protocol below.

**Interfaces:**
- Consumes: CLI, report schema and analyzer summary from Tasks 1–3.
- Produces: copyable Original commands for both directions and an evidence report containing every seed 42–44 run.

- [ ] **Step 1: Update terminology and commands in README**

State explicitly that branch names no longer select behavior. Add these visible
WSL commands from `openpilot/tools/sim/carla`:

```bash
VN_TRAFFIC_MODE=0 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case right_to_left --carla-scene-seed 42 \
  --carla-scene-duration 20 --no-experimental-mode

VN_TRAFFIC_MODE=0 bash run.sh all --openpilot-longitudinal \
  --carla-town Town04_Opt --carla-spawn-point 40 \
  --carla-scene motorcycle_weave --carla-scene-variant close_cut_in \
  --carla-scene-case left_to_right --carla-scene-seed 42 \
  --carla-scene-duration 20 --no-experimental-mode
```

Document that `VN_TRAFFIC_MODE=1` will later run the same command for the
enhancement comparison; it is outside this implementation plan.

- [ ] **Step 2: Run the focused tests and style checks**

```bash
uv run --extra carla --extra tools python -m unittest \
  openpilot.tools.sim.tests.test_motorcycle_weave \
  openpilot.tools.sim.tests.test_vn_traffic_manifest
uv run ruff check \
  openpilot/tools/sim/run_bridge.py \
  openpilot/tools/sim/bridge/carla/carla_bridge.py \
  openpilot/tools/sim/bridge/carla/carla_world.py \
  openpilot/tools/sim/bridge/carla/scenes/motorcycle_weave.py \
  openpilot/tools/sim/analyze_motorcycle_weave.py \
  openpilot/tools/sim/tests/test_motorcycle_weave.py \
  openpilot/tools/sim/tests/test_vn_traffic_manifest.py
git diff --check
```

Expected: all unit tests PASS, Ruff reports no errors, and `git diff --check`
prints nothing.

- [ ] **Step 3: Run the visible Original protocol without selecting runs**

Run both directions for seeds 42, 43 and 44. Change only `--carla-scene-case`
and `--carla-scene-seed`; keep map, spawn, duration, camera, Params, model and
bridge unchanged. After each run, analyze its exact report directory:

```bash
uv run --extra carla --extra tools python ../analyze_motorcycle_weave.py \
  ../../../../.carla/reports/motorcycle-weave-RUN_ID
```

Capture at least one timestamped visible clip per direction. Do not rerun or
discard a seed because Original avoided contact.

- [ ] **Step 4: Fill the Original evidence report**

Create `docs/reports/2026-09-23-close-cut-in-original-scene.md` with one row per
run and these exact columns:

```markdown
| Case | Seed | Report directory | Started | Entered | Actual gap (m) | Entry TTC (s) | Active cut-in contact | Other contact | Completed 20 s | Clip timestamp |
| --- | ---: | --- | ---: | ---: | ---: | ---: | --- | --- | --- | --- |
```

Below the table, record:

```markdown
- right_to_left contact rate: N/3
- left_to_right contact rate: N/3
- spawn overlap observed: yes/no
- front teleport observed: yes/no
- all actual entry gaps within 2.5–3.5 m: yes/no
- scene accepted: yes/no
```

Set `scene accepted: yes` only when each direction has contact in at least 2/3
runs, every run has exactly one start, no overlap/teleport occurred, and actual
entry gaps meet the design range. Keep all failed rows in the document.

- [ ] **Step 5: Freeze the scene or adjust only scene geometry before repeating the complete protocol**

If the acceptance fields fail, adjust only the `close_cut_in` values in
`SCENE_VARIANTS` or the motorcycle path controller. Do not change Original
Openpilot behavior or analyzer thresholds. Re-run all six seeds after any
geometry change; never combine rows from different scene hashes into one
acceptance set.

When accepted, record the final commit and the SHA-256 of
`motorcycle_weave.py` in the report. That commit/hash becomes the scene input
for the later enhancement plan.

- [ ] **Step 6: Commit documentation and evidence**

```bash
git add openpilot/tools/sim/carla/README.md \
  docs/reports/2026-09-23-close-cut-in-original-scene.md
git commit -m "docs: validate original close cut-in scene"
```

Do not push until the user reviews the captured clips and the evidence table.

## Self-Review Result

- Spec coverage: all scene-only requirements map to Tasks 1–4; enhancement implementation is intentionally a separate future plan.
- Placeholder scan: every implementation and validation step is concrete; no fill-in markers or undefined follow-up tasks remain.
- Type consistency: `scene_variant` flows CLI → bridge → world → scene; scheduler and report names remain consistent across tasks.
- Review focus: invalid variants, deferred events, no teleport/overlap, wrong-actor contact and unfinished collided events each have an owning test.
