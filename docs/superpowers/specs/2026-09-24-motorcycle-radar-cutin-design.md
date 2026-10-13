# CARLA radar-assisted cut-in prediction

Status: proposed design for review; implementation and collision avoidance are not yet verified.
Branch: `codex/motorcycle-radar-cutin`.

## Objective and evidence

Avoid contact in the existing 30 km/h close-cut-in scene by observing adjacent
objects and predicting overlap with the ego path before they become a stock
lead. Preserve the existing model weights and runtime artifacts. Both directions
must work, including alternating traffic. All work and validation remain in
visible CARLA launched through WSL.

The previous diagnostic runs had late/inconsistent model confirmation and a
model hypothesis around 16–18 m while the cut-in actor approached a roughly
3 m entry gap. These are different geometric quantities and actor association
is not proven: this is evidence to investigate missing/wrongly associated range,
not a calibrated numerical model bias. The runs also failed the scene lateral
tracking gate. They are not acceptance evidence.

## Approach selection

Prefer CARLA radar measurements plus camera-path geometry and temporal tracking.
This adds an independent range measurement without rebuilding the driving model.
It has the strongest engineering rationale among the approaches researched;
there is no measured probability of success yet.

Lower lead-probability thresholds and asymmetric probability filtering can reduce
dropouts but cannot independently resolve incorrect range or missing objects.
A separate camera detector/tracker is a possible later alternative, with larger
perception and runtime validation scope. Neither is included in this increment.

## Sensing and transport

1. Add an opt-in `VN_RADAR_CUTIN=1` simulation setting, default off. Require
   `SIMULATION=1`, `VN_TRAFFIC_MODE=1`, longitudinal active and experimental mode
   off before radar-derived constraints can affect planning.
2. Attach one forward CARLA `sensor.other.radar` at a fixed documented front
   mounting point. Initial configuration: 100-degree horizontal FOV, 10-degree
   vertical FOV, 40 m range, 20 Hz and 3000 points/second. Verify actual supported
   attributes and motorcycle returns before freezing these values. Record mount,
   configuration and effective frame rate in every manifest.
3. Use only sensor returns (range, azimuth, altitude and radial velocity), ego
   motion, timestamps and model output for decisions. Actor IDs, actor transforms,
   lane waypoints, scheduled cut-in timing and semantic ground truth are excluded
   from this path. Ego dimensions/mount calibration are fixed vehicle geometry.
4. Radar radial velocity is not longitudinal velocity. Verify its sign against
   stationary targets and known moving targets; estimate planar motion from
   temporal positions with ego translation/yaw compensation. Carry uncertainty.
5. Convert sensor points into a documented model-compatible frame and explicit
   front-bumper clearance. Unit tests establish signs in both directions and
   offsets between sensor, camera and bumper origins.
6. Use the existing unused `customReservedRawData0` service for a bounded,
   versioned simulation-only payload after checking for other publishers. Include
   frame, simulation time, host acquisition time, calibration and bounded returns.
   Do not publish to stock `radarTracks` or overwrite `radarState`. Planner's
   additional subscription is optional and does not invalidate stock services.
7. Extend trace serialization/replay for the binary Data payload, including
   explicit encoding, size checks and malformed-message rejection. Preserve old
   report compatibility. Record acquisition-to-planner age rather than refreshing
   old observations with a new publication timestamp.

## Tracking and path conflict

Cluster valid returns by position and radial velocity, filter ground returns,
and associate clusters across fresh frames using predicted position, motion and
uncertainty. Maintain bounded tracks with internal IDs; these are not actor IDs.
Initial limits are 200 returns/frame and 32 tracks with deterministic selection.
Record when limits truncate input so evaluation can detect lost coverage.

Confirm a track with at least three fresh observations spanning 0.1 seconds.
Duplicate frames do not confirm a track. Ambiguous association, non-finite data,
timestamp reversal, and implausible jumps reset confirmation. Missing observations
may retain an internal hypothesis but cannot sustain an active constraint beyond
250 ms from its last valid measurement; missing/invalid sensor data removes the
radar constraint and leaves the existing control path available.

Use `modelV2.position` for the ego corridor and evaluate each track's predicted
position in that same frame over two seconds. Account for ego motion and path
curvature. Require an inward lateral trend and predicted overlap. A parallel or
outward-moving object does not qualify solely because it is nearby. Use ego half
width, an explicit assumed object half width of 0.5 m, and a 0.2 m margin; radar
does not classify an object as a motorcycle. Bound extrapolation by uncertainty.

Camera lead hypotheses can corroborate a geometrically consistent track, but
model lead confidence is not required to initiate a radar-confirmed conflict.
Otherwise the design would inherit the present late-vision failure. Report
camera corroborated, radar/path only, or ambiguous association distinctly.

## Planner integration

Reuse the existing optional MPC cut-in obstacle input. Keep both stock leads and
the existing model predictor. Compute a conservative longitudinal obstacle for
each confirmed conflict only from its predicted overlap time onward. Reduce
multiple valid additional obstacles to a per-horizon minimum while retaining
source/track attribution. Radar expiry must not remove an independent model risk.

MPC produces the requested acceleration through the current controller and bridge.
Keep the current acceleration bounds and model artifacts. Record observation,
confirmation, predicted entry, MPC selection, requested deceleration, actual
acceleration/brake and contact times separately. Existing `lead2` source labeling
is supplemented with explicit radar/model attribution and internal track ID.

## Implementation boundaries

- CARLA acquisition and cleanup: `carla_world.py`, bridge/launcher configuration.
- Pure bounded decoding, clustering and tracking: new simulation radar module.
- Pure conflict prediction: new `vn_radar_cutin_policy.py`, composed with the
  existing predictor in `longitudinal_planner.py`.
- Optional subscription: `plannerd.py`; Data-aware replay: `planner_trace.py`.
- Diagnostics, manifests, run attribution and documentation: existing scene
  report/analyzer and CARLA README.

No existing unrelated work is committed or discarded as part of this proposal.
The branch starts from current HEAD and retains all local uncommitted work; a
branch alone is not an immutable snapshot of that work. Record a baseline hash
of all relevant dirty files before implementation and preserve replay evidence.

## Verification and acceptance

First verify sensor feasibility in visible CARLA: both cut-in directions produce
usable returns early enough, range/sign/mount conversions are correct, and nearby
static surfaces do not become moving inward tracks. Capture raw observations and
run prediction in observation-only mode before allowing it to affect MPC.

Test coordinate transforms, radial-versus-planar velocity, ego turning, clusters,
two adjacent objects, ambiguous/crossing tracks, dropouts/duplicates/staleness,
curve/parallel/outward controls, payload truncation and rotated-log replay.
Verify future-overlap obstacles cause earlier MPC deceleration, stock hazards
remain effective, and mode 0/feature off/outside simulation/experimental mode
preserve pre-change control outputs through full planner+MPC replay.

Develop on seeds 42–44. Freeze configuration before confirmation seeds 45–49,
two repeats per mode/direction, finite 30-second episodes. Include alternating,
isolated directions, empty road, parallel motorcycles and steady-lead controls.
Both compared modes acquire/log the same radar configuration, but Original
ignores the new service for control. Also retain an observation-disabled parity
run. Record that Enhancement has additional sensor information; improvement does
not demonstrate improved performance of the unchanged camera model.

Acceptance requires valid active runs and measured completed events of both
directions, zero Enhancement contact, deceleration before overlap, and synchronized
visual evidence. For Original contact cases seek at least 0.3 seconds earlier
deceleration. Controls must not show extra deceleration over 0.5 m/s² for longer
than 0.5 seconds without path conflict. Report clearance, actual acceleration,
progress, missed tracks, false conflicts, timing loss and all worst cases.

The existing scene tracking gate must pass to claim acceptance. A run failing
scene tracking, sensor coverage, active control or data completeness remains a
diagnostic. Fixing the scene controller, if needed, is applied equally to all
comparison modes without relaxing thresholds or altering close-cut-in geometry.
Unavailable video leaves visual acceptance pending. Sensor failure or persistent
contact is reported as failure, not hidden by threshold tuning on confirmation seeds.

## References

- CARLA radar sensor: https://carla.readthedocs.io/en/latest/ref_sensors/#radar-sensor
- FrogPilot adjacent/far tracking, inspected commit `7ed377b`:
  https://github.com/FrogAi/FrogPilot/blob/7ed377bce6b9ddba41793421d5765e9aee2124a8/selfdrive/controls/radard.py
- sunnypilot probability filtering, inspected commit `a5f4465`:
  https://github.com/sunnypilot/sunnypilot/blob/a5f44653d7f43ad57fef2f546f3916ec4cbf3c56/openpilot/selfdrive/controls/radard.py
