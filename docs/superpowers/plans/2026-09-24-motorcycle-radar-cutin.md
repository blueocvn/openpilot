# Motorcycle Radar Cut-in Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement these tasks in order. Each task has an observable result and a test cycle.

**Goal:** Give the simulation-only Phase 2 planner an earlier, measured cut-in obstacle when a motorcycle moves inward from either adjacent lane.

**Architecture:** CARLA radar publishes bounded point measurements through an unused optional simulation service. A pure tracker associates point clusters over time; a path-conflict predictor generates an optional future MPC obstacle. Original mode never consumes that obstacle. Existing scene ground truth is only used to evaluate detections.

**Tech Stack:** Python 3, CARLA 0.9.16, Cap'n Proto `Data`, Openpilot messaging, NumPy, pytest/unittest, existing rotated JSONL reports.

**Spec:** `docs/superpowers/specs/2026-09-24-motorcycle-radar-cutin-design.md`

## Global Constraints

- Work on `codex/motorcycle-radar-cutin`; preserve existing dirty files and actor-only evaluation data.
- Visible CARLA in WSL, 30 km/h, close-cut-in geometry unchanged, both directions tested.
- `SIMULATION=1`, `VN_TRAFFIC_MODE=1`, `VN_RADAR_CUTIN=1`, longitudinal active, experimental mode off gate planner effects.
- Do not rebuild/change model weights or runtime model artifact; do not inject actor IDs/poses or brake commands.
- Additional radar service is optional; Original and radar-off replay outputs must equal pre-change planner and MPC.
- Acquisition age limit 250 ms, three fresh confirmations spanning 100 ms, future overlap within two seconds.

## Review Focus

1. Empty or ground-only returns: no cut-in obstacle in empty-road control.
2. Nearby parallel motorcycle: no cut-in obstacle without inward motion.
3. Reused/late radar message: no confirmation or surviving obstacle past 250 ms.
4. Two objects crossing in depth: ambiguous association must not borrow the closer object's range.
5. Opposite-side return: preserve right-to-left/left-to-right sign after ego/camera/radar transforms.

## Files and responsibilities

- `openpilot/tools/sim/bridge/carla/carla_world.py`: radar spawn, frame acquisition, cleanup.
- `openpilot/tools/sim/lib/carla_radar.py`: pure point conversion, bounded payload validation/encoding.
- `openpilot/tools/sim/lib/simulated_sensors.py`: publish optional payload.
- `openpilot/selfdrive/controls/plannerd.py`: optional subscription and timestamp checks.
- `openpilot/selfdrive/controls/lib/vn_radar_cutin_policy.py`: tracking, conflict and obstacle.
- `openpilot/selfdrive/controls/lib/longitudinal_planner.py`: safety gate and merge with existing obstacle.
- `openpilot/tools/sim/planner_trace.py`: Data payload replay and source diagnostics.
- Existing sim/controls tests, scene report/analyzer, manifest and README: verification and usage.

### Task 1: Sensor measurement and bounded transport

**Interfaces:** `encode_radar(frame: int, timestamp_ns: int, points: list[tuple[float,float,float,float]]) -> bytes`; `decode_radar(payload: bytes) -> RadarFrame | None`. Point tuple is local x/y/z meters and radial velocity m/s.

- [ ] Write tests for left/right signs, front mount offset, invalid values, oversize truncation and versioned decode rejection; run them to verify failure.
- [ ] Implement pure conversion/codec, then add a CARLA radar sensor with fixed 20 Hz configuration and frame-matched queue. Record sensor metadata and sample gaps.
- [ ] Publish each acquired frame through `customReservedRawData0` only when requested in simulation. Verify no other publisher owns the service, and run sim tests.
- [ ] Verify WSL/CARLA smoke observation: returns for both crossing directions at usable range and no semantic actor information in payload. If not, stop before enabling planner effects and record why.

### Task 2: Adjacent-object tracker and conflict predictor

**Interfaces:** `RadarCutInPredictor.update(frame: RadarFrame | None, model, ego_speed_mps: float, model_time_ns: int, now_ns: int, active: bool) -> None`; `obstacle(horizon_s: np.ndarray) -> np.ndarray | None`; `diagnostic: dict`.

- [ ] Write failing tests for the five Review Focus cases plus curve, duplicate frame, data loss and two directions. Hand-check expected entry side/time and range.
- [ ] Implement bounded clustering (200 points, 32 tracks), ego-motion compensation, nearest plausible association with ambiguity reset, confirmation and 250 ms expiry.
- [ ] Compute the path corridor from `modelV2.position`, require inward motion and future overlap; return an obstacle beginning only at overlap time.
- [ ] Run controls and sim tests. In observation-only CARLA runs, compare predictions with scene events and actor ground truth; record first reliable detection time and false conflicts.

### Task 3: Planner gating and replay parity

**Interfaces:** optional Data service decoded before planner update; when radar threat exists, combine it with the existing model obstacle using per-horizon minimum, preserving independent threat attribution.

- [ ] Add failing tests for gated/no-data mode, stale/malformed radar, stock lead precedence, MPC future obstacle, and full old-planner/MPC mode-0 parity.
- [ ] Add optional subscription and planner integration. Do not change stock `radarState`, bridge actuation, solver structure or compiled artifacts.
- [ ] Extend rotated trace/replay to encode Data bytes deterministically and write radar source, age, track, conflict, selected obstacle and actual accel to diagnostics.
- [ ] Run targeted suites, full relevant controls/sim suites, lint and full planner/MPC replay on the same recorded inputs.

### Task 4: Visible evaluation and documentation

- [ ] Ensure manifest captures opt-in flag, sensor settings, runtime version and sampled coverage; analyzer reports missed/ambiguous tracks separately from `hasLead`.
- [ ] Run visible WSL seeds 42–44 with Original/Enhancement for both isolated directions, then empty/adjacent/steady controls. Fix only reproducible defects; do not tune to confirmation seeds.
- [ ] Freeze settings. Run seeds 45–49 twice each mode/direction for 30 seconds, recording every valid, invalid and contact run. Ask the user for synchronized clips only if available; unfilmed cases remain visually unaccepted.
- [ ] Report braking timing, source, actual deceleration, clearance/contact, false conflict, progress, scene-tracking validity, and all gate failures. Update README/architecture with exact run commands and limitations.
