# CARLA radar-assisted motorcycle cut-in — preliminary result

Status: implementation and diagnostic runs, **not accepted collision avoidance**.
Branch: `codex/motorcycle-radar-cutin`. Scope: visible CARLA in WSL only.

## Change

An opt-in CARLA radar now supplies raw, bounded range/angle/height/radial-speed
returns to a simulation-only tracker. A confirmed inward-moving adjacent object
may add a future obstacle to the existing longitudinal MPC when
`VN_RADAR_CUTIN=1` and `VN_TRAFFIC_MODE=1`. The model, compiled artifact,
stock `radarState`, actuator bridge and scene cut-in geometry were not changed.
Original (`VN_TRAFFIC_MODE=0`) ignores the added sensor for control. Full
planner/MPC replay parity against the stock reference passed when 507 captured
cycles with the optional radar service were replayed in Original mode.
The Original and alternating Enhancement reports have matching ONNX,
compiled-artifact and runtime-loaded artifact hashes; the model was not
rebuilt for this experiment.

## Visible diagnostic evidence

All runs used `Town04_Opt`, spawn 40, `close_cut_in`, 30 km/h scene cruise,
longitudinal engaged, experimental mode off, and visible CARLA. CARLA was
stopped after each test.

| Report directory suffix | Mode / case / duration | Completed cut-ins | Contact | Note |
| --- | --- | ---: | --- | --- |
| `460e0342d6a14b0390fd4005d7881536` | Original, right→left, 15 s | 1 | Yes | Radar captured only for observation. |
| `b5817c0af7104e0199adcfe17e47268e` | Original, left→right, 15 s | 1 | Yes | Radar captured only for observation. |
| `85de822221ca41e1abee640d2a3788a0` | Enhancement, right→left, 15 s | 1 | Second cut-in contacted | Tracker confirmed first but rejected noisy second returns; led to bounded-trend fix. |
| `b91e5d6743254741987c682dcc6b2961` | Enhancement, right→left, 15 s | 1 | None before episode end | Second cut-in incomplete; cannot count as success. |
| `ed11b3ce27c14495a3fc148009127834` | Enhancement, alternating, 30 s | 2, one each direction | None | Most useful preliminary result; scene gate still fails. |

In the final alternating run, the right→left event requested at least
−3.30 m/s², measured −3.58 m/s² ego acceleration and 0.052 CARLA brake
maximum. The left→right event requested at least −2.97 m/s², measured
−3.48 m/s² and 0.045 brake maximum. These are logged values, not visual
confirmation. Small brake-actuator values alone do not imply weak vehicle
deceleration. The radar predictor confirmed 23 and 11 planner cycles within
the respective three-second event windows. Both cut-ins completed without
contact in this one diagnostic run.

The alternating scene tracking gate **failed**: lateral tracking P95 was
0.757 m (required ≤0.4 m), while maximum was 0.757 m (required ≤0.8 m)
and speed-error P95 was 0.588 m/s (required ≤1 m/s). The old 10 Hz compact
actor record also omitted `bumper_gap_m`, making its pose evidence invalid;
that logging field is fixed for future runs, but these existing reports remain
diagnostic and are not relabeled valid. No synchronized user video was
available. Seed 45–49 confirmation runs, repeats, paired controls and
scene-tracking correction remain outstanding.

## Interpretation and next gate

Radar returns were visible before both directions entered the ego lane,
including before the previously collided second right→left event. This
supports earlier perception in this CARLA setup, not generalized motorcycle
detection. The tracker uses a fixed assumed object footprint and does not
classify motorcycles. Its false-conflict behavior needs empty-road,
adjacent-only and steady-lead controls across more seeds. The model-only
predictor can still fail independently. The 30-second no-contact result is
promising but not proof of a robust safety improvement or suitability for a
real car.

The current tracker uses radar radial speed as a bounded closing-speed proxy
at short range; a planar-velocity estimate and explicit ego-yaw compensation
on curves remain unvalidated. This is another reason to keep it simulation-only.

Next: correct the scene actor lateral tracking without changing cut-in
geometry, then repeat both modes under the same frozen scene/configuration.
Only after the tracking, data, provenance, parity and video gates pass should
the seed 45–49 confirmation set be scored for collision avoidance.

## Wrap-up checkpoint — 2026-10-13

The current implementation is being preserved on `codex/motorcycle-radar-cutin`
as a development checkpoint. It includes the preceding model-only cut-in
predictor, optional future MPC obstacles, startup acceleration smoothing,
CARLA radar acquisition and bounded transport, radar conflict tracking,
planner integration, replay diagnostics, and the compact actor gap logging
correction. Collision-avoidance acceptance remains pending.

Fresh verification in the existing Ubuntu WSL environment:

- The complete simulator test directory and all four Vietnam policy/planner
  test modules passed: **189 passed, 5 skipped**. The runner used the project's
  `.venv/bin/python` with the installed system pytest package appended to its
  import path. Two warnings remain: an unregistered `slow` marker and a camera
  view destructor accessing an uninitialized texture in a UI test.
- Ruff passed for all 23 changed or new Python source/test files.
- Original-mode replay of report
  `motorcycle-weave-460e0342d6a14b0390fd4005d7881536` against planner and MPC
  from `6a125380ac8e3b261ed1feb60a9eb8d6d84e18a5` passed for **440 cycles**.
  This is a separate fresh check from the earlier 507-cycle replay above.
- `git diff --check` passed for the simulator, controls and documentation
  changes. Live CARLA episodes were not rerun during this wrap-up.

The next validation work remains the scene tracking correction, complete pose
logging in new runs, false-conflict controls, planar motion/ego-yaw validation,
frozen configuration, seed 45–49 repeats, and synchronized visual evidence.
The existing raw simulation reports stay local under `.carla/reports`; their
diagnostic status is unchanged. Unrelated local configuration, scratch files,
and the existing AGNOS deletion are excluded from this checkpoint.
