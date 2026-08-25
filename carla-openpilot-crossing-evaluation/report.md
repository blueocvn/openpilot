# Báo cáo nghiên cứu: openpilot tích hợp CARLA trước tình huống xe cắt ngang

> **Bản ghi tại thời điểm đánh giá (commit `4304ea8`).** Nội dung, đường dẫn và
> các phát hiện bên dưới được giữ nguyên như lúc chạy đánh giá — kể cả các mô tả
> đã lỗi thời (ví dụ "Honda CAN": nhánh CARLA hiện dùng
> `openpilot/tools/sim/lib/carla_simulated_car.py` với DBC Tesla Model 3). Mã
> nguồn khi đó nằm ở gốc repo (`tools/`, `selfdrive/`), nay nằm dưới
> `openpilot/`. Đọc như tư liệu lịch sử, không phải mô tả cây mã hiện tại.

**Chủ đề:** CARLA-integrated openpilot response to motorcycle and passenger-car crossing hazards, including dataset design and ML policy retraining

**Trạng thái:** 3/14 hạng mục đã có kết quả nghiên cứu sâu và qua validator.

## Tóm tắt điều hành

Các artifact hiện có cho thấy bridge CARLA hiện chưa có đủ tính hợp lệ để dùng lỗi phanh dọc làm bằng chứng retrain policy: quyền longitudinal đang thuộc bridge-owned stock ACC, không phải openpilot. Cần sửa và xác nhận quyền actuator, đồng bộ thời gian/frame, tần số sensor, chuyển đổi tọa độ và mismatch vehicle model; sau đó chạy lại baseline trước khi đánh giá các ca xe cắt ngang.

Một kết quả `failed` chỉ được quy cho policy/planning khi scenario và simulator hợp lệ, input/model/perception đúng và đủ sớm, lệnh policy có quyền actuator thực tế, nhưng hành động vẫn vi phạm safety gate lặp lại. Các lỗi bridge, sensor, timing, perception hay control phải được sửa theo tầng tương ứng trước.

## Mục lục

1. [Integration architecture and timing fidelity](#integration-architecture-and-timing-fidelity) — Hoàn tất
2. [Baseline sanity and determinism](#baseline-sanity-and-determinism) — Hoàn tất
3. [Passenger car crossing left to right with clear visibility](#passenger-car-crossing-left-to-right-with-clear-visibility) — Hoàn tất
4. [Passenger car crossing right to left with clear visibility](#passenger-car-crossing-right-to-left-with-clear-visibility) — Chờ nghiên cứu
5. [Motorcycle crossing left to right with clear visibility](#motorcycle-crossing-left-to-right-with-clear-visibility) — Chờ nghiên cứu
6. [Motorcycle crossing right to left with clear visibility](#motorcycle-crossing-right-to-left-with-clear-visibility) — Chờ nghiên cứu
7. [Passenger car emerging from occlusion](#passenger-car-emerging-from-occlusion) — Chờ nghiên cứu
8. [Motorcycle emerging from occlusion](#motorcycle-emerging-from-occlusion) — Chờ nghiên cứu
9. [Scenario parameter coverage and automated test generation](#scenario-parameter-coverage-and-automated-test-generation) — Chờ nghiên cứu
10. [Safety comfort and pass-fail metrics](#safety-comfort-and-pass-fail-metrics) — Chờ nghiên cứu
11. [Failure attribution and retraining decision](#failure-attribution-and-retraining-decision) — Chờ nghiên cứu
12. [Dataset specification collection and scenario sampling](#dataset-specification-collection-and-scenario-sampling) — Chờ nghiên cứu
13. [Labeling ground truth and dataset quality assurance](#labeling-ground-truth-and-dataset-quality-assurance) — Chờ nghiên cứu
14. [Policy retraining validation and regression release gates](#policy-retraining-validation-and-regression-release-gates) — Chờ nghiên cứu

## 1. Integration architecture and timing fidelity

Nguồn artifact: `Integration_architecture_and_timing_fidelity.json`

### Identity and scope

#### item_name

Integration architecture and timing fidelity

#### category

System integration

#### objective

Define a traceable closed-loop contract from CARLA world state and sensor capture through openpilot perception, planning, control, and the command applied to CARLA.; Prove that frames, timestamps, coordinate systems, units, vehicle dynamics, actuator authority, message rates, latency, and logs are sufficiently faithful and reproducible before a crossing response is attributed to policy behavior.; Current static-audit verdict: policy evaluation is blocked until the identified timing, provenance, rate, coordinate, and longitudinal-authority gaps are corrected and verified in instrumented runs.

#### system_boundary

- **Included components:** - CARLA 0.9.16 server, Town10HD_Opt world, Unreal rendering, synchronous world stepping, vehicle and sensor actors, physics, ground truth, and RPC connection across Windows and WSL2.
- Workspace bridge files under tools/sim/bridge/carla and tools/sim/lib: world lifecycle, camera queues, RGB-to-NV12 conversion, VisionIPC, camera-state messages, GNSS/IMU and driver-monitoring simulation, Honda CAN and panda simulation, engagement, control arbitration, actuator scaling, and CARLA VehicleControl output.
- openpilot v0.11.1 commit 4df40d2c1946a57242230186edd073c4073060a6: manager, hardware/device state, card, calibration/localization, modeld, planning, controlsd, selfdrived, message bus, logger, and UI diagnostics.
- Host scheduling, GPU execution, WSL/Windows transport, clock domains, log production, and artifact lineage because they affect data age and reproducibility.
- **Actuator authority:** Lateral: openpilot carControl.actuators.steeringAngleDeg is converted by the bridge to normalized CARLA steering and therefore has effective authority while openpilot is active.; Longitudinal in the current configuration: openpilotLongitudinalControl is false. The bridge emulates stock ACC using a proportional controller targeting 8.0 m/s; openpilot model/planner acceleration is not applied. A longitudinal crossing failure therefore cannot be labeled an openpilot policy failure.; Manual keyboard input can override actuators while active. It must be disabled and logged false in all evaluation runs.
- **Excluded claims:** No real panda, production CAN timing, production camera exposure, real actuator plant, road/tire variability, driver response, or public-road safety is represented.; A bridge smoke test or stable visualization does not establish causal timing fidelity, policy quality, or sim-to-real validity.


### Experimental design

#### setup

- **Target architecture:** - Use CARLA synchronous mode with one and only one tick owner. Prefer a 0.01 s physics step so the 100 Hz openpilot control loop has one fresh plant state and one effective actuator interval per cycle; retain camera sensor_tick = 0.05 s for 20 Hz model input. If 0.05 s physics is retained, declare effective actuation as 20 Hz and do not claim 100 Hz closed-loop fidelity.
- For each 100 Hz cycle k: CARLA advances using command u[k-1]; the bridge receives snapshot/state and every sensor due for frame k; it atomically publishes vehicle state and camera data with original source frame/time; openpilot computes; the bridge selects one fresh u[k] before a deadline and applies it for the next tick. Log every transition.
- Wait for all sensors expected for a tick before advancing. Key queues by CARLA frame rather than accepting any frame greater than or equal to the current frame. GPU camera delay may be several frames, so frame identity, not callback arrival order, is authoritative.
- Apply synchronous/fixed-step/substep settings before reload, reload the world for every repetition, recreate actors deterministically with batched commands, and reseed every stochastic component after reload.
- **Camera contract:** - Road and wide CARLA image.frame, image.timestamp, image.transform, width, height, FOV, projection model, distortion, post-processing, and blueprint hash must survive into a bridge frame ledger.
- Map CARLA simulation capture time to host monotonic time using a run-level clock anchor, while preserving raw simulation time. Do not timestamp a frame after RGB-to-NV12 conversion and call that capture time.
- Populate roadCameraState and wideRoadCameraState frameId, frameIdSensor, timestampSof, timestampEof, processingTime, sensor identity, and transform consistently with VisionIPC metadata. If an instantaneous CARLA capture is represented with a synthetic exposure interval, document the exposure model; timestampEof must still map to the CARLA capture event.
- Use the openpilot camera model intentionally. The 40-degree 1928-pixel road camera implies about 2648-pixel focal length and matches the configured simulated road intrinsics closely. The current 120-degree pinhole wide camera implies about 557 pixels versus openpilot's approximately 567-pixel wide configuration; lens model and residual reprojection error must be measured, not assumed.
- Calibrate/verify camera mount translation and rotation. The current CARLA sensor mount is x = 0.8 m, z = 1.13 m and both cameras share it, while the published camera-state transform is an identity matrix. Define the semantics of that message field and separately preserve the full CARLA extrinsic.
- **Vehicle state contract:** - Create one immutable per-tick state snapshot consumed by simulated CAN, GNSS, IMU, and logging. Do not share a mutable SimulatorState across unsynchronized 100 Hz threads.
- Publish CAN/car state at declared rates from the same source frame. Each duplicate must be explicitly identified as a zero-order-held sample rather than given a new apparent measurement timestamp.
- Set IMU and GNSS sensor_tick explicitly, retain CARLA source frame/time/transform, and publish at intended openpilot service rates. The current loop publishes five IMU messages and ten GPS messages on each 100 Hz bridge iteration, which implies about 500 Hz IMU and 1000 Hz GPS, while services.py declares approximately 104 Hz IMU and 10 Hz GPS; this must be corrected.
- Separate ground truth from sensor estimates. CARLA true pose/velocity must not be silently used as a perfect measured value without declaring noise, latency, sampling, and coordinate conversion.
- **Actuator and vehicle contract:** - Log desired model action, planner output, carControl, carOutput, command selected by the bridge, normalized CARLA VehicleControl, the CARLA frame in which it becomes effective, and measured response.
- Identify the CARLA plant using steering and longitudinal step/chirp tests. Validate steering sign, hard-coded steer_ratio = 15, max wheel angle, clipping, gain, delay, rate limits, throttle acceleration mapping, brake deceleration mapping, and response at all tested speeds.
- Do not use Tesla Model 3 dynamics with Honda Civic CarParams as an implicit production-car surrogate. Either configure a validated abstract plant and matching CarParams or document this as a simulator-specific controller study and exclude vehicle-specific safety claims.
- If enabling openpilot longitudinal control, remove bridge stock-ACC authority, validate the current accel/1.6 throttle and -accel/4 brake conversions or replace them with an identified actuator model, then repeat all baseline and integration gates.
- **Instrumentation and logging:** Add a run ID and trace ID spanning CARLA frame, sensor frame, VisionIPC buffer, modelV2, longitudinalPlan, controlsState/carControl, bridge selection, VehicleControl apply, and next CARLA snapshot.; Log CARLA simulation time, host monotonic time, message logMonoTime, callback arrival, queue wait, conversion start/end, VisionIPC send/receive, model execution, planner/control publication, bridge read, command apply, and observed plant response.; Store full rlog rather than qlog for rate/timing analysis, plus camera videos/lossless audit frames, CARLA recorder, ground-truth Parquet/JSONL, process/resource logs, configuration manifest, and derived latency tables.

#### independent_variables

- **Configuration factors:** - Physics step: preferred 0.01 s versus legacy 0.05 s; camera capture period fixed at 0.05 s.
- Renderer/post-processing: headless low-postprocess and visible/high-quality as separate profiles.
- Camera topology: road only versus road plus wide; pinhole versus an explicitly selected CARLA wide-angle projection if adopted.
- Longitudinal authority: bridge-emulated stock ACC versus openpilot longitudinal; these are separate systems and must never share one score.
- Restart class, seed, CARLA/world reload, process cold/warm state, hardware/GPU/driver, and nominal versus controlled compute load.
- **Timing injections:** Camera callback delay, RGB-to-NV12 conversion delay, VisionIPC/message delay, CAN/state delay, control-command delay, packet/frame loss, reordered callback, and CPU/GPU contention. Inject one known fault at a time after the nominal contract passes.; Use bounded injections such as 0, 10, 25, 50, 100, 150, and 200 ms to verify that measured latency and failure detection recover the injected value and that stale data is rejected. These are project-proposed test points.
- **Coordinate and plant tests:** Vehicle yaw at 0, 90, 180, and 270 degrees; positive and negative unit translations/velocities on each CARLA axis; positive/negative roll, pitch, yaw, acceleration, angular velocity, steering, throttle, and brake.; Known 3D calibration targets across image center, corners, near/far range, road/wide cameras, and camera mount perturbations.; Speed and actuator sweeps matching the crossing campaign; dry/clear baseline first, then each weather and surface-friction profile as a separately qualified stratum.
- **Controlled constants:** Code/model/map/environment hashes, actor blueprints and physics parameters, camera intrinsics/extrinsics, weather, spawn/route, driver-monitoring simulation, Params, process set, and analysis software must remain frozen within one qualification group.

#### observables

- **Trace ledger:** run_id, trace_id, CARLA world frame and elapsed_seconds, source sensor type/frame/timestamp/transform, callback monotonic time, queue depth/drop count, conversion interval, VisionIPC frame/SOF/EOF, camera-state frame/SOF/EOF/logMonoTime, modelV2 frame/timestampEof/frameAge/frameDropPerc/modelExecutionTime/logMonoTime, plan/control logMonoTime, selected command time, apply frame, and effect frame.; Clock-anchor pairs and residuals for CARLA simulation time to host monotonic time. Wall-clock time is metadata only and must not be used for latency subtraction.
- **Vehicle and ground truth:** CARLA transform, bounding box, linear/angular velocity, acceleration, wheel state, physics control, applied VehicleControl, collision/lane events, map waypoint/lane frame, and every actor transform.; can, pandaStates, carParams, carState, livePose, liveCalibration, cameraOdometry, controlsState, selfdriveState, carControl, carOutput, modelV2, drivingModelData, longitudinalPlan, onroadEvents, managerState, procLog, logMessage, and errorLogMessage.
- **Camera and projection:** Raw image dimensions/FOV/projection/distortion/postprocess, focal length derived from CARLA projection, openpilot intrinsics selected by deviceState and sensor enum, camera extrinsics, calibration result, target reprojection error, road/wide pixel and timestamp alignment, checksums, and dropped/duplicated frames.; Current camera-state timestampSof/timestampEof values, which static inspection shows are not populated, and VisionIPC timestamps, which are currently assigned after conversion.
- **Rates and resources:** Per-service source sample rate, publication rate, unique-source-frame rate, inter-arrival distribution, maximum gap, validity/alive flags, queue occupancy, CPU/GPU utilization, memory, model execution time, and real-time factor.; Differentiate repeated publication of the same measurement from a new sensor sample; both counts are needed to expose the current IMU/GPS duplication.
- **Authority and causality:** carParams.openpilotLongitudinalControl, bridge openpilot_longitudinal flag, stock cruise state, manual_control_active, all candidate commands, arbitration winner, clipping/saturation, and why each command was accepted or rejected.; For every ego-state change, the most recent command that could causally affect it; for every policy output, the exact source camera and vehicle-state age used.

#### expected_safe_behavior

- Each CARLA physics frame has exactly one preceding effective control command, one atomic vehicle-state sample, and all due sensor samples; each openpilot output is traceable to one source frame and the next applied command.
- CARLA simulation time, source frame identity, and host monotonic time remain distinguishable and consistently mapped. No message is made to appear fresh by republishing stale source data with a new timestamp.
- Road and wide images for one model inference represent the same CARLA capture frame and correct camera geometry. No future frame, stale frame, or post-conversion timestamp is mislabeled as capture time.
- Coordinate transforms and units pass all basis/sign tests: meters, m/s, m/s^2, radians/s, degrees/radians, steering sign, curvature sign, NED velocity, gravity/down axis, and camera projection are unambiguous.
- The plant response matches the vehicle parameters and delays used by openpilot within the qualified envelope; saturation and arbitration are explicit.
- Nominal runs meet openpilot service rates and timing gates without process restart, dropped frame, queue overflow, accumulating latency, or real-time overrun.

#### relevant_openpilot_components

- **Input path:** - tools/sim/bridge/carla/carla_world.py: CARLA world settings, actor/sensor spawn, camera queues, state read, control application, reset, and tick.
- tools/sim/lib/camerad.py and simulated_sensors.py: RGB-to-NV12, VisionIPC and camera-state publication, IMU/GNSS, peripheral and synthetic driver monitoring.
- tools/sim/lib/simulated_car.py: Honda CAN and panda state publication and carParams/safety interaction.
- cereal/services.py and cereal/log.capnp: service rates and schemas for FrameData, modelV2, state, planning, control, and diagnostics.
- **Model and control path:** - selfdrive/modeld/modeld.py and fill_model_msg.py: VisionIPC receive, road/wide sync check, dropped-frame handling, camera warp selection, model execution, action delay compensation, and modelV2/drivingModelData/cameraOdometry timestamps.
- selfdrive/controls/plannerd.py and longitudinal_planner.py: model consumption, plan publication, modelMonoTime, and processingDelay.
- selfdrive/controls/controlsd.py, selfdrive/selfdrived, carControl/carOutput/controlsState/selfdriveState: control generation, engagement, alerts, and safety-related state.
- tools/sim/bridge/common.py: 100 Hz host loop, separate 100 Hz simulated-car thread, separate 20 Hz camera thread, CARLA tick every fifth host loop, arbitration, and command conversion.
- **Built in checks:** selfdrive/test/test_onroad.py provides useful native timing gates: camera SOF period near 50 ms with maximum deviation below 5 ms, sequential frame IDs, road/wide synchronization below 2 ms, EOF after SOF and within 50 ms, and modelV2 execution maximum below 60 ms with mean below 40 ms.; modeld flags road/wide timestamp differences above 10 ms, calculates frame drops from VisionIPC frame IDs, and may skip model evaluation after a dropped frame. The integration should satisfy the stricter onroad road/wide 2 ms test.; The current test_carla_closed_loop verifies only bridge start, engagement, and movement above 1 m/s; it is insufficient for timing or causal fidelity.


### Evaluation

#### applicable_standards_and_prior_work

- CARLA synchrony/time-step guidance is directly applicable: one tick client, synchronous fixed-step operation, sensor queues, waiting for expected sensor data, compatible physics substeps, reload for deterministic repetitions, and awareness that GPU camera data can be delayed by multiple frames.
- CARLA coordinate and sensor documentation defines the left-handed X-forward/Y-right/Z-up convention, meters/degrees, sensor frame/timestamp/transform, camera FOV and sensor_tick, IMU units, and compass/geographic orientation. These are the primary interface contract.
- The CARLA ROS bridge is useful prior work for lockstep design: in synchronous mode it waits for all expected sensor messages and can wait for vehicle control before advancing; it also distinguishes simulation time from system time.
- ASAM OSI provides a standardized simulation interface and explicit sensor/ground-truth reference-frame concepts. It is a useful schema reference even if the project retains native CARLA/openpilot messages.
- ISO 23150 provides logical-interface context between environmental sensors and fusion functions. It supports explicit semantic interfaces and metadata but does not by itself validate this bridge's timing or mechanics.
- openpilot's own schemas, services registry, modeld implementation, process-replay suite, and onroad timing tests are authoritative for the input/output and timing assumptions this simulator must satisfy.
- ISO 21448:2022 supports separating intended-function/performance insufficiency from integration and technical faults; policy retraining is inappropriate while interface fidelity remains unresolved.

#### pass_criteria

- **Status:** These are project-proposed gates except where explicitly inherited from openpilot's onroad tests. Current static audit does not pass them.
- **Contract and coordinates:** - All interface fields, units, axes, handedness, timestamp domains, frame IDs, sampling/hold behavior, authority, and saturation semantics are documented and enforced by automated tests.
- CARLA global-to-NED polar-vector mapping follows North = -CARLA Y, East = +CARLA X, Down = -CARLA Z. The present vNED code uses +Z and must be corrected. Angular-vector transforms are derived separately and verified by basis rotation tests rather than copied from polar-vector signs.
- All basis/sign/unit tests pass with numerical error <= 1e-5 for pure software transforms. Camera projection RMS <= 2 pixels, P95 <= 3 pixels, and max <= 5 pixels over the qualified target volume; no systematic sign or FOV bias.
- Vehicle/CarParams mismatch is resolved or explicitly scoped; measured steering and longitudinal delays differ from configured values by <= 20 ms and steady-state gain error is <= 10% within the qualified range.
- **Frame and timing integrity:** - Exactly one world-frame record per physics step; one atomic state sample per step; one road/wide pair for every due 20 Hz capture; zero skipped, duplicate, out-of-order, future, stale, or silently dropped source frame in an accepted run.
- Camera FrameData and VisionIPC IDs/timestamps agree. timestampEof represents mapped CARLA capture time, timestampSof is documented and earlier than EOF, and processing/publish times are separate.
- Inherit openpilot onroad gates: maximum camera SOF-period deviation from 50 ms < 5 ms; road/wide synchronization < 2 ms; EOF-SOF > 0 and < 50 ms; sequential frame IDs; modelV2 execution maximum < 60 ms and mean < 40 ms.
- Project end-to-end gate: capture-to-effective-command P95 <= 150 ms and max <= 200 ms, with no increasing latency trend. State source age at controls P95 <= one physics period plus 5 ms and max <= two physics periods.
- Real-time factor remains 1.00 +/- 0.02 over the analyzed interval; no physics/control deadline miss, queue overflow, process restart, or model frame drop.
- **Rates and causality:** Unique source samples and publication rates match declared services within 2% after warm-up: controls/car state near 100 Hz, model/cameras/plan near 20 Hz, IMU near 104 Hz, and GPS near 10 Hz. Stale-republication ratio is zero unless an explicitly modeled zero-order hold is labeled with its original source time.; Every applied command has exactly one arbitration record and a source openpilot output no older than the declared deadline; every measured response maps to the command active during its CARLA integration interval.; At least 10 deterministic reload and 5 cold-start runs meet all gates; any code/model/map/timing/hardware profile change requires requalification.
- **Authority gate:** A policy dimension may be judged only if its output is demonstrably connected to the actuator. The current configuration can judge openpilot lateral behavior, but longitudinal policy judgment fails this gate because bridge-emulated stock ACC owns throttle/brake.

#### fail_criteria

- **Hard integration failure:** - Any missing/duplicate/out-of-order/mismatched frame, fabricated freshness, timestamp-domain subtraction without validated mapping, camera queue overflow, sensor timeout, process restart, control applied to the wrong frame, or undocumented arbitration.
- Any coordinate, unit, sign, FOV, intrinsic/extrinsic, steering, or NED basis test failure; any plant delay/gain outside the qualified envelope.
- Any violation of native camera/model timing gates, service-rate gates, end-to-end latency, state-age, real-time-factor, or deterministic-repeatability limits.
- Any attempt to score longitudinal openpilot policy while openpilotLongitudinalControl is false.
- **Current static blockers:** - CARLA physics advances at 20 Hz while the bridge/control/CAN loop runs at 100 Hz, so five control computations share one plant state and only the latest command before a tick is effectively integrated.
- The loop reads vehicle state before the due world tick; camera data is read after the tick and published by an independent thread, creating an undocumented state-camera phase offset.
- Camera queues may discard old frames without a drop counter, the selector accepts image.frame >= current_frame, and the original CARLA frame/time is not propagated to openpilot logs.
- Camera FrameData timestampSof/timestampEof are left unset, while VisionIPC SOF/EOF are both assigned a host timestamp after CPU conversion rather than CARLA capture time.
- IMU and GNSS callbacks retain only the latest value without source frame/time; simulated publication duplicates them at approximately 500 Hz and 1000 Hz respectively under the 100 Hz loop instead of declared service rates.
- Mutable SimulatorState is accessed by independent threads without an atomic snapshot; vNED vertical mapping uses CARLA +Z instead of Down = -Z; IMU handedness/sign conversion is not explicit.
- Current reset teleports the vehicle rather than reloading the world, which does not meet CARLA's deterministic repetition guidance.
- Tesla physics, Honda CarParams/CAN, hard-coded steer ratio, and unvalidated longitudinal actuator scaling prevent vehicle-specific policy interpretation.
- **Invalid run:** Incomplete manifest/trace, resource overload, unexpected tick client, wrong version/hash, actor/spawn mismatch, manual input, logging loss, or failed clock fit. Retain and label invalid runs; do not score them as policy failures.
- **Escalation:** One hard integration failure blocks policy attribution for that configuration. The defect must be corrected and the full integration plus no-hazard baseline rerun before crossing tests resume.

#### failure_attribution

- **Simulator:** World frame/time or physics diverges before different commands, actor/physics initialization differs, or CARLA sensor frame/ground truth is internally inconsistent. Correct setup, reload, seeding, or CARLA configuration; do not retrain.
- **Bridge and timing:** CARLA source data is correct but frame join, clock mapping, queueing, conversion, publication rate, coordinate transform, CAN mapping, arbitration, or command application is wrong. Correct bridge/interface; do not retrain.; A stale or future camera/state input, post-conversion timestamp, duplicated sensor sample, or 20/100 Hz plant mismatch is an integration failure even if model output appears unsafe.
- **Perception model:** Only after source frame, geometry, capture time, calibration, and input bytes are verified may stable ground truth with incorrect model output be attributed to perception/model. Confirm with process replay and independent valid captures.
- **Policy planning:** Only after model input/output is correct may an unsafe action or plan be attributed to policy/planning, and only if the output has actuator authority. Current longitudinal plan has no throttle/brake authority.
- **Control vehicle:** Correct desired action and command but incorrect CARLA response indicates controller, scaling, delay, saturation, or plant mismatch. Fix/calibrate control or vehicle model; policy retraining is not the first response.
- **Causal evidence rule:** Attribution requires a single trace showing the first divergence from a matched passing/reference run. Downstream differences are consequences, not independent causes.


### Dataset and ML lifecycle

#### dataset_requirements

- **Purpose:** Create an immutable integration-conformance dataset before creating a policy-training dataset. Its unit is a complete causally traced run, not an isolated frame.; Include nominal no-hazard runs, coordinate/projection fixtures, actuator-identification runs, and controlled timing-fault injections.
- **Modalities:** Road/wide image bytes and metadata; CARLA source sensor data and snapshot ground truth; camera calibration targets; CAN/panda/state; model/plan/control; command arbitration; VehicleControl; plant response; process/resource logs; CARLA recorder; full rlog; and the joined trace ledger.; Preserve raw and mapped timestamps, clock anchors, queue counters, source and publication rates, all coordinate transforms, intrinsics/extrinsics, actor physics, code/model/config hashes, and derived metrics.
- **Coverage:** At least 10 reload plus 5 cold-start nominal runs per exact configuration, four yaw orientations and positive/negative basis inputs, calibration targets spanning image/range, actuator tests at every crossing speed, and each injected delay/loss class.; Separate 0.01 s and 0.05 s physics, renderer profiles, camera projection, longitudinal authority, host hardware, and software revisions; do not pool timing statistics across them.
- **Splits and lineage:** Split by run/configuration, never frame. Freeze golden contract fixtures and untouched cold-start acceptance runs; do not use them to tune bridge constants or ML.; Manifest every artifact with run ID, parent scenario, hashes, versioned schema, machine/GPU/driver, restart class, seeds, clock fit, actor/sensor/plant configuration, and checksum.
- **Storage:** Use content-addressed immutable storage; keep raw evidence separate from reproducible derived tables. Retain failed and invalid attempts in an audit partition.; Full rlog is mandatory because qlog decimates key services and cannot prove timing fidelity.

#### labeling_and_quality

- **Labels:** Run validity, architecture pass/fail, first-divergence component, longitudinal authority, physics/control rate, renderer, restart class, and remediation version.; Per trace: source/due/published/consumed/applied/effect frame; raw and mapped times; coordinate frame/unit; new sample versus held/duplicated sample; valid/stale/drop/reorder; arbitration winner; saturation; and ground-truth response.
- **Automated quality:** Schema/checksum validation, monotonic frame/time, complete one-to-one joins, expected sample counts, clock-fit residual, finite/range checks, actor inventory, exact configuration hashes, and independent recomputation of coordinate/projection/latency metrics.; Synthetic fixtures with known frame delay, coordinate transform, and actuator response must be recovered within one timestamp resolution and numeric tolerance.
- **Review:** Visually inspect calibration fixtures and first/middle/last camera pairs; inspect every anomaly trace. Two reviewers approve any attribution that could cause model/policy retraining.; Do not interpolate across missing frames, backfill capture time from publication time, or relabel a duplicate as a new source sample.
- **Leakage prevention:** Keep all frames from one run/config lineage in one split. Acceptance traces and fault-injection values remain hidden from tuning.; If an acceptance artifact is used to tune timing, calibration, controller constants, or thresholds, retire it from holdout and capture a new independent acceptance set.

#### retraining_implications

- **Default:** No policy/model retraining is justified while any architecture or timing gate fails.
- **Minimum trigger:** All integration and no-hazard baselines pass; source images/state, coordinate geometry, clock mapping, frame identity, rates, latency, actuator model, and authority are verified; the same model/policy defect is reproduced in at least three independent valid cold starts and offline replay.; The first divergence occurs in model/policy output rather than simulator, bridge, timing, control, or plant, and appears on an untouched holdout or at least two independent capture strata.; For longitudinal retraining, openpilotLongitudinalControl must be true and the policy/planner output must be shown to cause the applied brake/throttle command.
- **Integration defects not training data:** Stale/misaligned frames, incorrect timestamps, duplicated GNSS/IMU, coordinate/sign mistakes, camera intrinsics mismatch, process overrun, CAN/safety mismatch, bridge stock-ACC behavior, and uncalibrated Tesla/Honda plant response must be fixed, not learned around.; Do not add corrupted integration frames to training; quarantine them with defect labels.
- **After trigger:** Curate verified source-domain examples, preserve temporal sequences and action delay, adjust sampling/objectives only for the demonstrated failure mode, and maintain independent real-domain and simulation holdouts.; Rerun architecture, no-hazard, all crossing scenarios, and regression suites after any model or policy change.

#### validation_and_regression

- **Static and unit:** Lint/schema checks and code-level contracts for clocks, frames, rates, transforms, units, camera intrinsics, BGRA-RGB-NV12 conversion, steering/curvature signs, NED mapping, arbitration, and actuator clipping.; Assert service publication and unique-source rates. A test must fail the current five-IMU/ten-GPS-per-loop duplication and unpopulated camera timestamps.
- **Software in loop:** Run coordinate/projection fixtures, camera frame barriers, controlled delay/loss injections, and actuator identification before closed-loop crossing tests.; Extend test_carla_closed_loop to assert joined trace completeness, camera/state rates, native onroad timing limits, authority, continuous engagement, deterministic repetition, and complete artifacts.
- **Offline replay:** Replay frozen VisionIPC/log inputs through modeld/planner/control and compare frame IDs, actions, timing-independent outputs, and tolerance-bounded floating-point fields. Replay validates component repeatability but not closed-loop causal timing.; Use CARLA recorder for diagnosis, not as a substitute for a rerun where controller outputs affect the world.
- **Release and rollback:** Architecture gate, no-hazard baseline, then crossing scenarios is the mandatory order. Any architecture failure blocks downstream interpretation.; Version and freeze thresholds before acceptance. Preserve the prior complete environment, bridge, model, map, Params, and golden traces for rollback.; Any CARLA/Unreal, bridge, openpilot/model, DBC/CarParams, camera, timing, plant, OS/GPU driver, or longitudinal-authority change triggers requalification.


### Decision support

#### risks

- Policy misattribution is the primary risk: current longitudinal motion is bridge-controlled, and current timing/provenance defects can manufacture apparent perception or policy errors.
- The 20 Hz CARLA physics step makes the apparent 100 Hz controller operate repeatedly on unchanged state, hiding the effective zero-order hold and distorting delay/control dynamics.
- Post-conversion host timestamps understate capture-to-decision latency; missing FrameData timestamps prevent independent reconstruction from rlog.
- Accepting a later camera frame and silently dropping queue entries can introduce future/stale data while logs still show sequential synthetic frame IDs.
- Republishing one IMU/GNSS sample many times with new message times can overweight stale measurements and overload localization/messaging while appearing high-rate.
- Mixed CARLA simulation time, monotonic time, and wall time can produce negative or fictitiously low latency if subtracted directly.
- CARLA left-handed Z-up versus openpilot right-handed Z-down and NED conversions create silent sign errors; angular vectors require special care under handedness change.
- Road intrinsics happen to align closely, but wide pinhole versus physical fisheye behavior, identity metadata transform, and unverified calibration can shift object geometry and policy response.
- Tesla physics plus Honda CAN/CarParams and a hard-coded steering ratio can yield internally consistent steering messages while actual curvature/delay is wrong.
- Synchronous CARLA can be deterministic in simulation time while openpilot processes still depend on wall-monotonic scheduling; real-time overruns change the integrated system.
- Perfect synthetic CAN, fake driver monitoring, and near-noiseless sensors omit production delay, noise, faults, and takeover behavior.
- Exact tested configuration may not transfer across hardware, renderer, CARLA build, weather, speed, or real roads.

#### recommendations

- 1. Declare the current bridge not ready for policy attribution and freeze crossing-result interpretation until the blockers below are closed.
- 2. Refactor to a clear lockstep cycle with 0.01 s physics, 0.05 s camera sensor_tick, one tick owner, per-frame sensor barrier, atomic state snapshot, one effective command per physics step, and explicit deadlines.
- 3. Introduce a frame/clock ledger from CARLA source through VisionIPC, model, plan, control, bridge, command application, and plant response. Preserve CARLA frame/time and map it to monotonic capture time before conversion.
- 4. Stop accepting image.frame >= current frame; require the exact expected frame, count every discarded/late frame, and populate camera-state plus VisionIPC metadata consistently.
- 5. Correct sensor scheduling: publish unique IMU near 104 Hz and GNSS at 10 Hz from timestamped source samples, retain hold provenance, and protect shared state with an atomic snapshot or single-threaded frame object.
- 6. Implement and test explicit CARLA-to-openpilot polar and angular coordinate transforms, including NED Down = -CARLA Z; add four-yaw basis/sign/unit tests.
- 7. Calibrate road/wide projection with known 3D targets, document lens model/extrinsics, and meet <= 2-pixel RMS reprojection before judging crossing-object geometry.
- 8. Identify the CARLA steering and longitudinal plant at all test speeds, align CarParams/delays or adopt a documented abstract plant, and remove hidden Tesla/Honda assumptions.
- 9. Decide longitudinal scope. For policy-retraining conclusions, enable verified openpilot longitudinal authority and rerun architecture plus no-hazard qualification; otherwise label results as openpilot lateral plus bridge stock-ACC only.
- 10. Enforce native openpilot camera/model timing checks, project end-to-end latency limits, full-rate logging, deterministic reloads, controlled fault injections, and at least 10 reload plus 5 cold-start passing runs before the crossing campaign.

#### sources

- **Title:** Synchrony and time-step
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** One tick client, synchronous fixed step, sensor queues/barriers, GPU camera delay, substeps, reload and seed requirements for deterministic simulation.
- **Title:** Coordinates and transformations
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/coordinates/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** CARLA left-handed X-forward, Y-right, Z-up coordinates, meters, degrees, actor frames, and transform semantics.
- **Title:** Sensors reference
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/ref_sensors/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Sensor frame/timestamp/transform, camera FOV and sensor_tick, IMU units and compass orientation, GNSS, and projection options.
- **Title:** CARLA ROS bridge synchronous-mode configuration
- **Publisher or author:** CARLA Simulator ROS bridge documentation
- **Url:** https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Simulation-time use, waiting for all expected sensors, optional wait for vehicle control, passive tick ownership, and reproducible synchronous operation.
- **Title:** openpilot camera transformations at workspace baseline
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot/blob/4df40d2c1946a57242230186edd073c4073060a6/common/transformations/camera.py
- **Publication date:** 2026-05-29 commit baseline; accessed 2026-08-06
- **Supported claims:** openpilot device/road/view frames, simulator camera intrinsics, and projection transforms.
- **Title:** openpilot modeld at workspace baseline
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot/blob/4df40d2c1946a57242230186edd073c4073060a6/selfdrive/modeld/modeld.py
- **Publication date:** 2026-05-29 commit baseline; accessed 2026-08-06
- **Supported claims:** VisionIPC frame synchronization, frame-drop behavior, camera warp selection, model delay compensation, execution timing, and modelV2 publication.
- **Title:** openpilot message schemas and service registry at workspace baseline
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot/blob/4df40d2c1946a57242230186edd073c4073060a6/cereal/log.capnp
- **Publication date:** 2026-05-29 commit baseline; accessed 2026-08-06
- **Supported claims:** FrameData and modelV2 frame/timestamp/timing fields, device-frame model coordinates, plan/control fields; nominal service rates are in cereal/services.py at the same commit.
- **Title:** openpilot onroad timing tests at workspace baseline
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot/blob/4df40d2c1946a57242230186edd073c4073060a6/selfdrive/test/test_onroad.py
- **Publication date:** 2026-05-29 commit baseline; accessed 2026-08-06
- **Supported claims:** Native camera period, frame continuity, road/wide synchronization, exposure timing, and model execution-time gates.
- **Title:** Workspace CARLA bridge implementation audit
- **Publisher or author:** Project workspace based on comma.ai openpilot v0.11.1
- **Url:** file:///D:/work/openpilot/tools/sim/carla/IMPLEMENTATION_LOG.md
- **Publication date:** 2026-08-05 to 2026-08-06
- **Supported claims:** CARLA/openpilot versions and local integration intent. Static findings were verified directly in carla_world.py, carla_bridge.py, common.py, camerad.py, simulated_sensors.py, simulated_car.py, run_bridge.py, and test_carla_bridge.py.
- **Title:** ASAM OSI User Guide 3.4.0
- **Publisher or author:** Association for Standardization of Automation and Measuring Systems
- **Url:** https://www.asam.net/fileadmin/Standards/OSI/ASAM_OSI_User-Guide_V3.4.0.html
- **Publication date:** 2026 release series; accessed 2026-08-06
- **Supported claims:** Simulation ground-truth and sensor-data interface/reference-frame concepts.
- **Title:** ISO 23150:2023 - Road vehicles — Data communication between sensors and data fusion unit for automated driving functions — Logical interface
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/83293.html
- **Publication date:** 2023-05
- **Supported claims:** Logical and semantic interface context between environmental sensors and data-fusion functions.
- **Title:** ISO 21448:2022 - Road vehicles — Safety of the intended functionality
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/77490.html
- **Publication date:** 2022-06
- **Supported claims:** Distinguishing functional/performance insufficiency in sensor-dependent ADAS from other defects and structuring verification/validation evidence.


## 2. Baseline sanity and determinism

Nguồn artifact: `Baseline_sanity_and_determinism.json`

### Identity and scope

#### item_name

Baseline sanity and determinism

#### category

System integration

#### objective

Demonstrate that the CARLA server, CARLA-openpilot bridge, simulated sensors and CAN, openpilot processes, and CARLA vehicle dynamics form a stable closed loop before any vehicle or motorcycle crossing actor is introduced.; Establish a no-hazard reference trajectory and quantified run-to-run repeatability envelope. Crossing-scenario results are admissible only when this gate passes.; Expose whether longitudinal motion is controlled by openpilot or by the bridge, so a later braking failure is attributed to the component that actually had actuator authority.

#### system_boundary

- **Inside:** - CARLA 0.9.16 server physics, rendering, map, weather, ego blueprint, sensor actors, world tick, and ground-truth state.
- The workspace CARLA bridge, including RGB conversion, camera queues, VisionIPC publication, simulated GNSS/IMU, simulated Honda CAN and panda state, engagement handshake, manual-override logic, actuator conversion, and bridge process scheduling.
- openpilot v0.11.1 at commit 4df40d2c1946a57242230186edd073c4073060a6, including manager supervision, model inference, calibration/localization, planning, self-drive state, vehicle interface, lateral control, logging, and alerts.
- The CARLA Tesla Model 3 ego vehicle dynamics, while the CAN interface presented to openpilot is a simulated 2022 Honda Civic radarless interface. This vehicle-interface/dynamics mismatch is part of the integration under test and must be recorded as a validity risk.
- For the current baseline configuration, openpilot lateral commands have actuator authority, but longitudinal authority is outside openpilot: AlphaLongitudinalEnabled is false and the bridge emulates stock ACC with a bounded speed controller targeting 8.0 m/s.
- **Outside:** - Real vehicle sensors, production CAN buses, panda hardware timing, tire/road variability, actuator delays, driver behavior, and physical safety validation.
- Motorcycle or vehicle crossing behavior; the baseline contains no hazard actors and only proves readiness for those tests.
- Certification or evidence that the integration is safe for public-road use.
- Longitudinal openpilot policy performance in the current configuration, because model or planner acceleration is not applied to the CARLA vehicle when openpilotLongitudinalControl is false.
- **Decision boundary:** A baseline failure blocks interpretation of all crossing runs. A baseline pass proves integration consistency only within the recorded simulation configuration; it is not proof of crossing-object safety or real-world safety.


### Experimental design

#### setup

- **Configuration freeze:** Pin CARLA server and Python client to 0.9.16; pin openpilot to commit 4df40d2c1946a57242230186edd073c4073060a6; hash all modified and untracked bridge files, model artifacts, DBC files, Params, launch scripts, CARLA map/OpenDRIVE content, and scenario configuration.; Record OS/WSL version, CPU, GPU, GPU driver, renderer mode, process affinity if used, CARLA command line, Python environment lock hash, and whether the run is warm-reset or a full cold restart.; Use one authoritative client to tick the CARLA world. No second synchronous client may call tick.
- **Deterministic carla initialization:** - Enable synchronous_mode and fixed_delta_seconds = 0.05 s, explicitly set substepping = true, max_substep_delta_time = 0.01 s, and max_substeps >= 5 so fixed_delta_seconds <= max_substep_delta_time multiplied by max_substeps.
- Apply synchronous settings before the world used for a repetition is loaded or reloaded, reload the world for every repetition, and recreate actors with batched synchronous commands. A transform-only reset is not sufficient for the determinism gate.
- If Traffic Manager is used later, put it in synchronous mode and reset the same random_device_seed after every reload. The no-hazard baseline should not use Traffic Manager actors.
- Set ClearNoon explicitly after every reload. Spawn only the hero vehicle and required sensors. Persist blueprint attributes and vehicle physics-control values in the manifest.
- Attach CARLA collision and lane-invasion sensors even though the current bridge does not provide them, and log their CARLA frame and simulation timestamp.
- **Sensor and bridge integrity:** - Run dual cameras because the supported launcher requests dual_camera. Keep sensor_tick = 0.05 s. Record both the original CARLA image.frame and the openpilot camera frameId; require an explicit one-to-one mapping instead of discarding the CARLA frame identifier.
- Require each road and wide image used for a tick to have image.frame equal to the current CARLA world frame. Do not silently accept a later frame. Record queue overflow, stale-frame discard, timeout, conversion duration, and VisionIPC publication timestamp.
- Record CARLA snapshot elapsed_seconds and delta_seconds, monotonic host time, camera SOF/EOF time, model receipt/output time, plan/control time, and actuator-application time in one trace so simulated-time determinism and wall-time performance are not conflated.
- Prohibit keyboard/joystick input during measurement and log the manual_control_active flag as false.
- **Execution:** - Before measurement, run the existing RGB unit test and closed-loop smoke test; confirm manager, modeld, controlsd, selfdrived, card, calibration/localization, logger, and bridge are alive and error-free.
- Allow stationary initialization, then require selfdriveState.active continuously for 5 simulated seconds before starting the 60 simulated-second measurement interval. Exclude initialization and engagement transients from tracking and comfort aggregates but report them separately.
- Engineering gate: run at least 10 world-reload repetitions with the same seed/configuration and 5 full cold-start repetitions. Release gate: run 60 valid cold-start repetitions with zero hard safety/integration failure.
- Save the CARLA recorder log, full openpilot rlog, road and wide video or lossless audit frames, ground-truth tables, run manifest, process logs, and derived metric file under one immutable run ID.

#### independent_variables

- **Held constant for determinism:** CARLA and Python API version, openpilot commit and model hashes, bridge hash, map/OpenDRIVE hash, spawn transform, route, ego blueprint and physics parameters, camera geometry and post-processing, weather, fixed time step and substeps, renderer, Traffic Manager disabled, actor population, simulated CAN vehicle, cruise target, and host hardware/software stack.; Initial ego pose, linear and angular velocity, control state, ignition, calibration state, cruise/engagement sequence, analysis start condition, and 60 s simulated duration.
- **Controlled factors:** - Restart class: world reload versus full CARLA, bridge, and openpilot cold restart.
- Seed: use one fixed seed for strict repeatability. A secondary seed-invariance check may use at least three documented seeds even though no stochastic non-ego actor is present.
- Renderer profile: visible/high-quality and headless/low-postprocess must be separate configurations; never aggregate them because image pixels and compute load differ.
- Compute-load stress: nominal load and a separately identified resource-stress run may be compared, but only nominal runs enter the primary deterministic envelope.
- **Future baseline matrix:** After the initial gate, repeat the no-hazard baseline at each ego speed planned for crossing experiments and at each camera/weather profile. Do not infer a pass at 8 m/s covers another speed or visibility condition.; If openpilot longitudinal control is enabled, treat it as a new system configuration, remove the bridge speed controller from actuator authority, and rerun the entire baseline before crossing tests.

#### observables

- **Carla ground truth:** World frame, simulation timestamp and delta, ego transform, center-of-mass pose, bounding box, linear/angular velocity, acceleration, wheel and vehicle physics parameters, applied VehicleControl, nearest driving waypoint, lane ID, lane width, lane-center lateral offset, heading error, collision events and impulses, lane-invasion events, and actor inventory.; For each camera: CARLA frame, timestamp, transform, intrinsics/blueprint attributes, queue arrival time, conversion time, selected/discarded status, and content checksum for diagnostic comparison.
- **Openpilot and bridge:** roadCameraState, wideRoadCameraState, liveCalibration, livePose, cameraOdometry, modelV2, drivingModelData, longitudinalPlan, carState, carControl, carOutput, controlsState, selfdriveState, onroadEvents, carParams, pandaStates, can, sendcan, managerState, procLog, logMessage, and errorLogMessage.; Preserve logMonoTime, validity, alive/frequency status, source frame IDs, desired path/curvature/acceleration, actual speed/acceleration/steering, actuator commands, control state, engagement/disengagement reasons, alert text, process restarts, CPU/GPU timing, and bridge last_controls/openpilot_longitudinal/manual override state.; Record stock_cruise_speed = 8.0 m/s and carParams.openpilotLongitudinalControl. This boolean determines whether longitudinal observations evaluate openpilot or only the bridge-emulated stock controller.
- **Derived events:** Initialization complete, first valid camera pair, first valid model output, controls engageable, engagement, measurement start/end, any message gap, frame mismatch, process restart, alert, disengagement, lane crossing, collision, manual override, and timeout.; A traceable causal timeline from CARLA tick through camera/model/plan/control to the command applied at the next CARLA tick.

#### expected_safe_behavior

- The ego initializes without collision or overlap, openpilot processes become healthy, engagement occurs reproducibly, and the system remains active for the entire measurement interval without alert, process restart, or manual intervention.
- The ego follows the verified straight lane without touching a lane boundary, oscillation, divergent steering, or spontaneous lane change. Heading, lateral position, speed, and actuator commands converge to stable values after the warm-up period.
- The current bridge-owned longitudinal controller approaches and maintains 8.0 m/s smoothly. This behavior is expected from the integration, not evidence that openpilot would brake for a crossing object.
- Every CARLA tick has the expected 0.05 s simulated-time increment; road and wide camera frames are paired to that tick; required openpilot streams are valid, in order, and near their declared service rates; model/control latency does not accumulate across the run.
- Repeated runs from an identical manifest remain within the declared trajectory, speed, yaw, actuator, and timing envelopes.

#### relevant_openpilot_components

- **Interfaces:** - cereal/services.py declares nominal full-stream rates: carState, carControl, controlsState, and selfdriveState at 100 Hz; modelV2, drivingModelData, longitudinalPlan, roadCameraState, and wideRoadCameraState at 20 Hz; liveCalibration at 4 Hz. Use full rlog rather than qlog when checking these rates because qlog is intentionally decimated.
- tools/sim/lib/camerad.py converts CARLA RGB into the device-compatible padded NV12 layout and publishes VisionIPC plus camera-state messages.
- tools/sim/lib/simulated_sensors.py publishes camera, IMU, GNSS, peripheral, and synthetic driver-monitoring data. Its host monotonic and wall-clock timestamps are not CARLA simulation timestamps, so explicit frame/time correlation is required.
- tools/sim/lib/simulated_car.py publishes simulated Honda CAN and pandaStates and consumes carControl, controlsState, carParams, and selfdriveState.
- **Decision and control:** modeld/modelV2 and drivingModelData for predicted path, motion, and model action stability; liveCalibration/livePose/cameraOdometry for geometry and ego-motion consistency; longitudinalPlan for planner output; controlsd outputs in carControl, controlsState, and carOutput; selfdrived state and onroadEvents for engagement and fault evidence.; The current CarlaBridge declares alpha_longitudinal_enabled = false. SimulatorBridge therefore applies openpilot steering but uses its own proportional throttle/brake controller around 8.0 m/s when carParams.openpilotLongitudinalControl is false.; The existing closed-loop test checks started, active, and vEgo > 1.0 m/s only. Extend it with frame integrity, sustained engagement, route tracking, collision/lane events, deterministic replay, and artifact assertions.
- **Diagnostics:** managerState and procLog expose process liveness and resource problems; logMessage/errorLogMessage and onroadEvents expose faults; pandaStates, can, sendcan, carParams, and selfdriveState expose vehicle-interface and safety-configuration mismatch.; Use route replay or a deterministic log-replay comparison only as a component diagnostic. CARLA recorder playback is not a substitute for closed-loop reruns because playback moves recorded actors rather than re-evaluating the controller's causal effect.


### Evaluation

#### applicable_standards_and_prior_work

- CARLA's official synchrony/time-step guidance is directly applicable. It says precision requires synchronous mode with a fixed step, compatible physics substeps, synchronous settings before loading/reloading, world reload for every repetition, batched commands, and a reset Traffic Manager seed when Traffic Manager is used.
- CARLA's recorder, Python API, and benchmarking guidance support per-frame ground truth, collision/lane-invasion event logging, replay for diagnosis, and explicit FPS/compute-load characterization. Recorder replay is diagnostic and must not be confused with deterministic closed-loop re-execution.
- ISO 11270:2014 provides lane-keeping system functionality and test-procedure context. It also makes clear that lane keeping support is not automatic driving and driver responsibility remains; project lane-error thresholds below are engineering proposals, not quoted certification limits.
- ISO 15622:2018 provides ACC control, diagnostics, and test-procedure context, but its highway/free-flow scope and limited requirements for stationary or slow objects do not establish pass criteria for urban crossing hazards. In the current bridge it does not validate openpilot longitudinal policy because the bridge owns longitudinal actuation.
- ISO 21448:2022 applies to functional insufficiencies in perception/processing and intended functionality for sensor-dependent ADAS. Its separation of technical malfunction from functional insufficiency supports the required attribution gate before retraining.
- ISO 34502:2022 supplies a scenario-based safety-evaluation framework, but its stated ADS and limited-access-highway scope is not directly the openpilot urban crossing use case; use the process concepts, not a claim of conformance.
- ASAM OpenSCENARIO can encode repeatable actor/scenario actions and OpenDRIVE describes road geometry. Their use improves scenario portability and traceability, while CARLA execution details and hashes remain necessary.
- openpilot's SAFETY.md defines openpilot as supervised ACC and automated lane centering, describes driver override and constrained actuation, and identifies software-in-the-loop/hardware-in-the-loop/in-vehicle testing. This baseline is one software-in-the-loop layer only.

#### metrics

- **Validity and integrity:** - Tick integrity: count delta_seconds values not equal to 0.05 s within numerical tolerance; count duplicate, skipped, or out-of-order world frames. Report count and longest gap.
- Camera integrity: for each world tick, count missing road/wide frames, CARLA-frame mismatches, road-wide pairing mismatches, queue overflows, stale discards, and timeouts. Report counts per run and per 1,000 expected pairs.
- Message health: for each required service, observed messages divided by expected messages over simulated time, median/P95/P99 inter-arrival interval, maximum gap, invalid-message count, and out-of-order logMonoTime count. Compare with declared full-service rates, not qlog rates.
- Process health: process restart count, error/critical log count, maximum CPU/GPU/memory load, and real-time factor equal to simulated duration divided by wall duration. A slow real-time factor is diagnostic unless it produces data loss or violates a project latency limit.
- **Closed loop tracking:** - Lateral error e_y in meters: signed dot product of ego reference point minus nearest lane-center waypoint with the waypoint right vector. Report mean, RMS, P95 absolute, and maximum absolute e_y after warm-up.
- Heading error in degrees: wrapped ego yaw minus lane-center yaw. Report mean, RMS, P95 absolute, and maximum absolute error.
- Speed error in m/s: carState.vEgo minus 8.0 m/s for the current bridge-owned longitudinal controller. Report mean absolute, RMS, P95 absolute, maximum absolute, overshoot, and settling time into plus or minus 0.5 m/s.
- Control tracking: desired versus measured curvature/lateral acceleration and commanded versus applied steering, throttle, and brake; report RMS/P95/max error and saturation fraction.
- Smoothness: longitudinal and lateral acceleration in m/s^2 and jerk in m/s^3. Derive with a versioned low-pass/differentiation method and report RMS, P95, P99, and max; never compare jerk from different filters.
- **Safety events:** Collision count and maximum collision normal impulse in N s; lane-invasion count and lane markings crossed; disengagement, takeover, control mismatch, excessive-actuation, calibration, and camera/model fault counts.; Time active divided by measurement time, time to first valid camera pair/model output/engagement, and count/duration of any inactive interval.
- **Repeatability:** Align repetitions by CARLA simulation timestamp with no dynamic-time warping. For each run against a designated reference, calculate RMS and maximum Euclidean position difference, absolute lateral-offset difference, speed difference, yaw difference, applied-control difference, and model path/action difference.; Report both within-world-reload and cold-start envelopes. Use median, P95, maximum, and the worst run ID; also report pairwise envelopes so reference-run selection cannot hide an outlier.; For hard-failure incidence, zero failures in n independent valid cold starts gives a one-sided exact 95% upper probability bound of 1 minus 0.05 raised to the power 1/n. At n = 60 this is approximately 4.87%. Do not claim a lower failure rate from only 10 repetitions.

#### pass_criteria

- **Status:** The numeric limits below are project-proposed engineering gates, not regulatory thresholds. They must be frozen before examining crossing outcomes and tightened after initial characterized data; loosening requires documented review and rerunning all affected scenarios.
- **Per run hard gates:** - Manifest complete and identical to the approved configuration; world reload used; no non-ego actor; analysis interval is 60 simulated seconds after 5 seconds of continuous active state; no manual input.
- Zero collision, zero lane invasion, zero process restart, zero error-level integration event, zero disengagement/inactive sample, zero safety mismatch, and zero actuator NaN or uncommanded saturation during the measurement interval.
- Every CARLA step advances exactly one frame and 0.05 s; zero out-of-order/duplicate frame; zero missing camera pair; every selected road and wide image has CARLA frame equal to the current world frame; zero camera timeout or queue overflow.
- For carState, carControl, controlsState, and selfdriveState, observed full-log count is at least 98% of 100 Hz expectation and no inter-arrival gap exceeds 30 ms. For road/wide camera state, modelV2, drivingModelData, and longitudinalPlan, count is at least 98% of 20 Hz expectation and no gap exceeds 150 ms. Any implementation-known lower rate must be declared and separately justified before the test.
- After warm-up: lateral-error RMS <= 0.30 m, P95 absolute <= 0.50 m, and maximum absolute <= 0.75 m while the full ego footprint remains within the lane; P95 absolute heading error <= 2.0 degrees and maximum <= 4.0 degrees.
- For the bridge-owned 8.0 m/s longitudinal loop after settling: mean absolute speed error <= 0.30 m/s, P95 absolute <= 0.50 m/s, and maximum absolute <= 1.0 m/s; P99 absolute longitudinal acceleration <= 1.5 m/s^2 and P99 absolute jerk <= 3.0 m/s^3. These longitudinal limits do not score openpilot policy.
- P99 absolute lateral acceleration <= 1.5 m/s^2 and P99 absolute lateral jerk <= 3.0 m/s^3 on the verified straight segment; steering command saturation fraction = 0%.
- **Repeatability gates:** - All 10 same-seed world-reload runs and all 5 cold-start engineering runs pass every per-run hard gate.
- Across same-manifest repetitions, pairwise RMS ego-position difference <= 0.05 m and maximum <= 0.20 m; pairwise speed-difference RMS <= 0.10 m/s and maximum <= 0.30 m/s; pairwise maximum yaw difference <= 0.25 degrees.
- Measurement-start engagement time differs by no more than 0.25 s across runs; derived P95 lateral error, P95 speed error, and P99 jerk each have a robust coefficient of variation <= 10%, unless their median magnitude is below the metric's numerical noise floor, in which case use the absolute envelopes above.
- Pixel checksums may be retained as diagnostics, but exact RGB equality is not a pass gate across GPU/renderer builds. Model input geometry, frame identity, and closed-loop outputs are the safety-relevant repeatability gates.
- **Release gate:** Before treating the crossing campaign as release evidence, obtain 60 valid cold-start baseline repetitions with zero hard failure and all repeatability limits satisfied. This supports only an upper 95% failure-probability bound of approximately 4.87% for the tested configuration.; Any change to CARLA/Unreal, map, renderer, GPU driver, openpilot code/model, bridge, simulated vehicle, vehicle physics, camera transform/intrinsics, timing, Params, longitudinal authority, or control calibration invalidates the baseline and requires rerun.

#### fail_criteria

- **Hard failure:** Any collision, lane invasion, unexplained disengagement, safety/control mismatch, process crash/restart, non-finite command, uncommanded actuator saturation, or violation of lane/speed/smoothness limits in an otherwise valid run.; Any same-manifest trajectory, speed, yaw, model, or actuator divergence beyond the repeatability envelope.; Any frame/data loss or synchronization violation that reaches model/control, or required stream validity/rate below its pass gate.
- **Invalid run not scored as policy failure:** Incomplete manifest, wrong version/hash, world not reloaded, unexpected actor, occupied spawn, manual input, route not straight or insufficient length, map/lane ground-truth defect, camera frame mismatch, bridge timeout, logger loss, or host resource exhaustion that causes missed data.; CARLA/server crash, sensor creation failure, actor spawn failure, or a test-orchestrator error. Invalid runs must be repeated after cause correction and must remain visible in the audit ledger; they may not be silently deleted.
- **Campaign escalation:** One hard baseline failure immediately blocks crossing-result interpretation and opens a defect with the complete causal trace.; Any invalid-run rate above 5% in the most recent 20 attempts is itself an integration reliability failure, even if valid runs pass.; Do not average a hard safety failure away. Pass requires every valid run to meet hard gates; aggregate statistics characterize margin and repeatability only.

#### failure_attribution

- **Simulator or scenario:** Ground-truth state diverges before different commands are applied, world frame/delta changes, initial transforms/physics differ, actor inventory differs, or CARLA recorder and snapshot show a map/physics/spawn anomaly. Correct CARLA initialization or scenario generation; do not retrain.; If transform-only reset diverges but reload-world repetitions do not, attribute the failure to reset methodology, consistent with CARLA determinism guidance.
- **Bridge or integration:** CARLA ground truth is stable but camera frame mapping, timestamps, calibration/intrinsics, CAN state, safety configuration, message rates, process lifecycle, or command conversion differ. Correct the bridge/interface and rerun baseline; do not retrain.; If throttle/brake behavior fails while carParams.openpilotLongitudinalControl is false, attribute primary responsibility to the bridge-emulated stock ACC and vehicle dynamics, not the openpilot longitudinal policy.; If desired openpilot steering is consistent but applied CARLA steering differs, inspect sign, steering ratio, max wheel angle, clipping, and Tesla/Honda dynamics mismatch.
- **Perception or model:** Identical, correctly synchronized camera inputs and valid calibration produce unstable model paths/actions, or model output departs from lane ground truth before planning/control diverges. Reproduce with log replay and cold-start closed loop, verify model artifact determinism, then consider model/data remediation.; A rendering-domain mismatch may be a SOTIF/perception limitation rather than a software fault. It still requires controlled evidence across independent valid runs before changing training data.
- **Policy or planning:** Model perception/path evidence is stable and adequate, but planned desired curvature/acceleration is unstable or unsafe. Confirm the relevant plan has actual actuator authority. Under the current stock-longitudinal baseline, acceleration-plan errors cannot explain applied throttle/brake.; Only after deterministic inputs, valid interfaces, and actuator authority are proven may a repeated decision defect be labeled policy/planning and considered for retraining.
- **Control or vehicle dynamics:** Desired plan/control is repeatable, but carControl/carOutput tracking or CARLA motion is not. Inspect controller tuning, rate/timing, steering conversion, saturation, simulated vehicle physics, and actuator delay; retraining policy is not the first action.
- **Minimum evidence bundle:** For every failure, retain the run manifest, reference-run diff, CARLA ground truth and recorder, raw frame-to-world mapping, full rlog and video, model/plan/control trace, applied CARLA commands, process/resource logs, and an attribution decision signed by integration, controls, and ML reviewers.


### Dataset and ML lifecycle

#### dataset_requirements

- **Purpose and unit:** This is primarily an evaluation and regression dataset, not automatically a training dataset. The atomic sample is a complete run; frames from one run are never treated as independent statistical trials.; Collect at least 10 same-seed reload runs plus 5 cold starts for engineering, and 60 valid cold-start runs for the release gate, each with a 60 s analyzed interval and preserved warm-up. At 20 camera pairs/s, 60 release runs yield about 72,000 analyzed road/wide frame pairs.
- **Modalities:** Synchronized road and wide RGB/video, optional lossless audit frames, camera metadata, CARLA ground-truth ego pose/velocity/acceleration, nearest lane geometry, collision/lane events, full vehicle controls, CARLA recorder log, full openpilot rlog, model/planner/control messages, CAN/panda state, calibration/pose, process logs, and resource telemetry.; A frame-index table joining run ID, CARLA world frame/time, road and wide CARLA image frame, VisionIPC/openpilot frameId, monotonic timestamps, modelV2 receipt/output, plan/control message times, and applied-actuator world frame.
- **Coverage and balance:** Primary stratum: one exact no-hazard configuration for strict determinism. Do not mix renderer, hardware, speed, weather, camera, map, vehicle, or longitudinal-authority profiles in one repeatability estimate.; Secondary strata should mirror every crossing-campaign speed, weather, camera profile, and longitudinal-authority configuration with no hazard. Keep equal run counts per stratum and preserve cold-start/reload labels.; Retain failed and invalid attempts in separate indexed partitions so dataset curation cannot hide integration instability.
- **Splits and lineage:** Group splits by complete run and configuration lineage, never by random frame. Near-duplicate repeated frames from a run must stay together.; Suggested regression split: frozen golden reference runs, development runs used to tune integration, and untouched acceptance cold starts. If later used for ML, reserve runs from distinct captures and at least one renderer/weather/map stratum as an external holdout.; Manifest fields include immutable run ID; parent campaign; UTC time; seed; restart class; all code/model/config/map hashes; CARLA/Unreal/Python API version; machine/GPU/driver; actor blueprints and physics hashes; sensor transforms/intrinsics; Params; rates; weather; route; and artifact checksums.
- **Storage:** Use immutable, content-addressed artifacts with a machine-readable schema/version, checksums, provenance, data-use classification, and retention policy. Store derived metrics separately from raw evidence so they can be recomputed.; Preserve full-rate logs for timing validation. A qlog alone is insufficient because services are deliberately decimated.

#### labeling_and_quality

- **Labels:** Run-level labels: valid/invalid, baseline pass/fail, hard-failure type, restart class, longitudinal authority, and final failure-attribution class.; Frame-level labels: CARLA frame/time, ego pose/bounding box/velocity/acceleration, lane ID/width/center offset/heading error, collision/lane-invasion state, active state, camera-pair integrity, model/plan/control availability, and applied actuator.; No-hazard label is generated from actor inventory plus verified route occupancy, not inferred only from the image.
- **Ground truth and quality checks:** Use CARLA snapshot, map waypoint/OpenDRIVE, actor transforms, collision sensor, and lane-invasion sensor as simulator ground truth. Independently recompute lane offset from stored transforms and map geometry.; Automated acceptance checks require complete manifest, artifact checksums, monotonically increasing frames/timestamps, exact frame joins, expected sample counts, finite values, actor inventory, route validity, and agreement between CARLA speed and carState.vEgo within a frozen tolerance.; Visually audit the first/middle/last paired frames and every anomalous run; verify camera FOV/orientation, lane rendering, occlusion absence, and that the ego is on the intended lane. Two reviewers adjudicate any attribution that could trigger ML retraining.
- **Uncertainty handling:** Do not force-label a frame when map waypoint selection is ambiguous, CARLA ground truth disagrees with visible lane markings, or camera/control frame association is missing. Mark the run invalid with a reason and exclude it from pass statistics while retaining it in the audit ledger.; Report numerical precision, filtering method, timestamp source, and interpolation. Never interpolate across a missing frame to make an integrity gate pass.
- **Leakage prevention:** Never split adjacent frames or repeated captures of the same run across ML training and validation. Group by run, route segment, configuration, and scenario lineage.; Acceptance runs and golden regression routes remain unavailable for hyperparameter selection or threshold tuning. Any reuse moves them out of holdout status and requires a new holdout capture.

#### retraining_implications

- **Default decision:** Do not retrain from a baseline failure. First correct simulator, timing, frame alignment, vehicle-interface, actuator-authority, or control problems and rerun the baseline.
- **Trigger for ml investigation:** All baseline integrity and deterministic gates pass; inputs, calibration, frame mapping, and model artifact hash are confirmed; the defect is reproduced in at least three independent valid cold-start runs and in offline log replay; and ground truth shows a model path/action defect before planning/control divergence.; The same defect appears on an untouched configuration/route holdout or across at least two independently captured visual strata, reducing the chance that one CARLA rendering artifact is being memorized.; For policy retraining specifically, prove the failing policy output has actuator authority. With current openpilotLongitudinalControl = false, longitudinal applied-control failures cannot trigger openpilot policy retraining.
- **Possible changes after trigger:** Add curated no-hazard negative examples only if the model shows false obstacle/stop behavior or unstable path prediction with verified images; include varied but physically consistent CARLA appearance and matched real-world-domain data to reduce simulator overfitting.; Adjust loss or sampling to penalize false braking, path oscillation, or temporal inconsistency only after checking that labels and causal timing are correct. Preserve safety constraints and do not train directly against bridge-controller artifacts.; Keep baseline data in a small, explicitly weighted regression slice; do not allow abundant straight-road frames to overwhelm rare crossing-hazard examples.
- **Not a trigger:** A transform-only reset difference, message drop, frame mismatch, process fault, wrong camera geometry, CAN/safety mismatch, Tesla/Honda dynamics mismatch, controller tracking error, or bridge stock-ACC speed error.; A single failed run, a failed invalid run, or a change observed only after pass thresholds were tuned on the same acceptance data.

#### validation_and_regression

- **Offline:** Run schema/checksum validation, frame-join validation, metric recomputation, and deterministic log replay of model/planner/control components using frozen artifacts. Compare model path/action and controls to golden envelopes with explicit tolerances.; Unit-test BGRA-to-RGB conversion, RGB-to-NV12 layout, camera-pair timestamps, CARLA/openpilot frame mapping, speed/steering sign and units, actuator clipping, and reset/reload orchestration.
- **Closed loop:** Keep the existing startup/engagement/movement smoke test, then add a 60 s no-hazard baseline test with collision/lane sensors, stream-rate assertions, continuous active state, route metrics, and stored artifacts.; Run 10 deterministic reload repetitions in CI on capable CARLA hardware and a scheduled 60-cold-start qualification suite for releases. Separate visible/high-quality and headless profiles.; After any ML or policy change, run baseline first, then all clear-view and occluded vehicle/motorcycle crossing cases. A baseline regression blocks the entire suite.
- **Safety and release gates:** No candidate may improve crossing metrics by violating baseline lane keeping, comfort, engagement, or timing gates. Compare candidate and incumbent on the same frozen holdout plus newly generated independent seeds/configurations.; Require zero new hard failures, no statistically or practically meaningful degradation of baseline margins, complete failure-attribution review, and reproducible artifacts before promotion.; Version the pass thresholds and evaluation code. Approval must identify whether openpilot or bridge/stock longitudinal had authority.
- **Rollback:** Retain the incumbent code, model, bridge, Params, environment lock, and golden dataset. If any post-promotion baseline or crossing regression appears, restore the complete incumbent configuration rather than only the model file.; A changed CARLA, renderer, GPU driver, map, camera, vehicle dynamics, DBC, or longitudinal-authority configuration requires requalification and cannot borrow a previous pass.


### Decision support

#### risks

- False confidence: a deterministic no-hazard pass establishes integration readiness, not motorcycle/vehicle crossing safety and not real-world performance.
- Authority mismatch: current openpilot longitudinal control is disabled. Any claim that a crossing test evaluates openpilot braking policy would be invalid unless authority is changed and re-baselined.
- Reset nondeterminism: the current bridge reset teleports the actor without reloading the world, whereas CARLA recommends world reload for reproducible repetitions.
- Synchronization masking: the current camera selector accepts an image whose CARLA frame is later than the current world frame and discards original frame identity before publishing, so skipped frames can be hidden without added instrumentation.
- Clock-domain error: CARLA simulation time, host monotonic time, GPS wall time, and openpilot logMonoTime are mixed; incorrect latency conclusions are likely without a frame/time join table.
- Vehicle-model mismatch: Tesla physics with Honda CAN, steering ratio assumptions, and bridge actuator scaling can dominate trajectory behavior and limit transfer to a supported real car.
- Synthetic sensor limitations: idealized/fake GNSS, IMU timing, driver monitoring, CAN, and calibration do not exercise production noise, delay, faults, or driver takeover.
- Renderer and compute nondeterminism: GPU rendering and asynchronous host processes may vary even when CARLA physics is deterministic; exact pixel equality is neither guaranteed nor the sole relevant criterion.
- Map-ground-truth risk: CARLA lane-invasion sensing depends on OpenDRIVE and may disagree with visible markings; route validation and visual audits are required.
- Threshold overfitting: choosing tolerances after seeing acceptance runs or crossing failures invalidates the claimed gate.
- Statistical overclaim: a small number of repeated passes gives a weak failure-rate bound; 60 zero-failure runs still only bound failure probability below about 4.87% at one-sided 95% confidence for the exact tested configuration.
- Sim-to-real gap: CARLA appearance, traffic behavior, physics, latency, and motorcycle dynamics differ from reality; retraining solely on simulator data can reduce real-world robustness.

#### recommendations

- 1. Freeze and emit a complete run manifest, including openpilot/bridge/model/map hashes and the longitudinal-authority boolean, before collecting any crossing data.
- 2. Fix the repetition lifecycle to follow CARLA determinism guidance: set synchronous/fixed-step/substep settings, reload the world for every repetition, recreate actors deterministically, and seed every stochastic component.
- 3. Add collision and lane-invasion sensors plus an explicit CARLA-world-frame to road/wide frame to model/plan/control/applied-command trace. Require exact frame pairing and expose queue drops instead of accepting later frames.
- 4. Extend the current smoke test into the stated 60 s no-hazard engineering gate and execute 10 reload plus 5 cold-start repetitions before the first crossing test.
- 5. Decide project scope explicitly: either evaluate the current system as openpilot lateral plus bridge-emulated stock longitudinal, or enable openpilot longitudinal authority. If the project goal is to decide whether openpilot policy needs retraining for crossing response, enable and verify openpilot longitudinal control, then rerun this entire baseline.
- 6. Validate that Town10HD_Opt spawn 16 leads to a straight, actor-free, sufficiently long segment; if not, create an explicit route and spawn transform rather than relying on index order.
- 7. Store full rlogs, videos/audit frames, CARLA recorder and ground truth, process logs, manifests, and derived metrics for every attempt, including invalid and failed runs.
- 8. Freeze project-proposed numeric gates before examining crossing outcomes, then qualify with 60 valid cold starts for release-level evidence. Report margins and the exact one-sided failure bound, not only pass/fail.
- 9. Use the causal attribution ladder simulator -> integration -> perception/model -> policy/planning -> control/dynamics. Retraining is permitted only after deterministic valid evidence reaches the model/policy trigger and the relevant output has actuator authority.
- 10. After baseline qualification, add a matching no-hazard control for every crossing-test speed, weather, camera, map, and longitudinal-authority stratum, and keep all holdout runs isolated from tuning.

#### sources

- **Title:** Synchrony and time-step
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Synchronous fixed-step operation, substep constraint, initialization-before-reload, per-repetition world reload, batched commands, and Traffic Manager seed requirements for physics determinism.
- **Title:** CARLA Python API reference
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/python_api/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** World settings, sensor frame/timestamp fields, collision events, lane-invasion events, actor state, and recorder interfaces.
- **Title:** Retrieve simulation data
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/tuto_G_retrieve_data/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Recording, querying, replay, sensor data collection, and use of the recorder as a tracing tool.
- **Title:** Benchmarking Performance
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/adv_benchmarking/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Benchmarking synchronous/fixed-step configurations and reporting average and standard deviation of simulation performance.
- **Title:** openpilot SAFETY.md at v0.11.1 workspace baseline
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot/blob/4df40d2c1946a57242230186edd073c4073060a6/docs/SAFETY.md
- **Publication date:** 2026-05-29 commit baseline; accessed 2026-08-06
- **Supported claims:** openpilot is supervised ACC and automated lane centering; driver override and actuator constraints; software-in-the-loop, hardware-in-the-loop, and in-vehicle test layers; ISO 11270/15622 context.
- **Title:** openpilot cereal service registry at v0.11.1 workspace baseline
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot/blob/4df40d2c1946a57242230186edd073c4073060a6/cereal/services.py
- **Publication date:** 2026-05-29 commit baseline; accessed 2026-08-06
- **Supported claims:** Nominal message frequencies and qlog decimation for carState, controlsState, selfdriveState, camera states, modelV2, drivingModelData, longitudinalPlan, calibration, and diagnostic streams.
- **Title:** Workspace CARLA integration code and implementation log
- **Publisher or author:** Project workspace; based on comma.ai openpilot v0.11.1
- **Url:** file:///D:/work/openpilot/tools/sim/carla/IMPLEMENTATION_LOG.md
- **Publication date:** 2026-08-05 to 2026-08-06
- **Supported claims:** CARLA 0.9.16 integration, fixed 0.05 s world step, Town10HD_Opt/ClearNoon defaults, dual camera bridge, simulated Honda CAN with Tesla dynamics, engagement lifecycle, and bridge-owned 8 m/s stock-longitudinal emulation. Verified against carla_world.py, carla_bridge.py, common.py, simulated_car.py, simulated_sensors.py, and test_carla_bridge.py in the same workspace.
- **Title:** ISO 11270:2014 - Intelligent transport systems — Lane keeping assistance systems (LKAS) — Performance requirements and test procedures
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/50347.html
- **Publication date:** 2014-05
- **Supported claims:** Lane-keeping basic control, diagnostics and test-procedure context; lane keeping support is not automatic driving and driver responsibility remains.
- **Title:** ISO 15622:2018 - Intelligent transport systems — Adaptive cruise control systems — Performance requirements and test procedures
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/71515.html
- **Publication date:** 2018-09
- **Supported claims:** ACC basic control, functionality, diagnostics and test procedures; highway/free-flow scope and limited treatment of stationary or slow objects.
- **Title:** ISO 21448:2022 - Road vehicles — Safety of the intended functionality
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/77490.html
- **Publication date:** 2022-06
- **Supported claims:** Framework for hazards caused by intended-function specification or performance insufficiencies in sensor/algorithm-dependent safety functions and ADAS.
- **Title:** ISO 34502:2022 - Road vehicles — Test scenarios for automated driving systems — Scenario based safety evaluation framework
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/78951.html
- **Publication date:** 2022-11
- **Supported claims:** Scenario-based safety-evaluation process and its stated limited-access-highway ADS scope.
- **Title:** ASAM OpenSCENARIO
- **Publisher or author:** Association for Standardization of Automation and Measuring Systems
- **Url:** https://www.asam.net/standards/detail/openscenario/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Machine-readable dynamic driving-scenario description and parameterization for repeatable virtual testing.
- **Title:** ASAM OpenDRIVE
- **Publisher or author:** Association for Standardization of Automation and Measuring Systems
- **Url:** https://www.asam.net/standards/detail/opendrive/
- **Publication date:** Continuously maintained; accessed 2026-08-06
- **Supported claims:** Logical road-network geometry representation used to support traceable lane and route definitions.


## 3. Passenger car crossing left to right with clear visibility

Nguồn artifact: `Passenger_car_crossing_left_to_right_with_clear_visibility.json`

### Identity and scope

#### item_name

Passenger car crossing left to right with clear visibility

#### category

Core crossing scenario

#### objective

Establish whether the CARLA-integrated openpilot system avoids or safely yields to an unobstructed passenger car that crosses the ego path from left to right, and determine the boundary of safe behavior over ego speed, crossing-vehicle speed, initial time to conflict, range, and crossing angle. The result must also identify whether any failure belongs to simulation, bridge timing and actuation, model inference, policy-planning, or low-level control before retraining is considered.

#### system_boundary

- **Inside:** - CARLA 0.9.16 world dynamics, ego Tesla Model 3 dynamics, crossing passenger-car actor, intersection geometry, collision and ground-truth state
- The Windows CARLA to WSL2 openpilot bridge, including RGB camera capture, RGB-to-NV12 conversion, VisionIPC publication, simulated CAN/panda, GNSS/IMU publication, synchronization, engagement, actuator conversion, and logging
- openpilot v0.11.1 at repository commit 4df40d2, including modeld, the driving vision and policy models, modelV2, longitudinal planning, controlsd, carControl, simulated vehicle interface, and safety state
- Scenario generation, repeatability controls, metric computation, dataset capture, and failure attribution
- **Outside:** Real-vehicle sensor hardware, production vehicle brake hydraulics, tire behavior outside the calibrated CARLA model, driver supervision, regulatory approval, and public-road deployment; Occluded crossings, motorcycles, right-to-left crossings, pedestrians, traffic-signal compliance, adverse visibility, and multi-actor interactions; these belong to other research items; Claims that a CARLA pass certifies real-world safety or Euro NCAP performance
- **Decision boundary:** A run is evidence about openpilot longitudinal policy only when an openpilot longitudinal mode is active and its commanded acceleration is actually converted to CARLA throttle and brake. In the current repository, CarlaBridge sets alpha_longitudinal_enabled=False and stock_cruise_speed=8.0 m/s, while bridge/common.py uses a bridge-owned proportional speed loop whenever carParams.openpilotLongitudinalControl is false. Therefore the current configuration is suitable for integration and shadow-policy observation, but it is not a valid closed-loop test of learned longitudinal policy. Any crossing failure in that configuration must be attributed first to test configuration or emulated stock ACC, not to the policy.

#### assumptions

- Use the repository baseline documented locally as openpilot v0.11.1 commit 4df40d2 and CARLA 0.9.16; freeze these identifiers, the CARLA package hash, map hash, Python environment, model-file hashes, bridge configuration, and vehicle physics in every result manifest.
- The crossing vehicle is visible from the first evaluated frame, lies inside the calibrated road or wide-road camera field of view, and has no physical or rendered occluder. Quantify visibility using projected ground-truth pixels rather than relying only on the scenario label.
- The ego follows a straight lane-center path through an uncontrolled junction. The crossing passenger car follows a constant-speed straight path from ego-left to ego-right and, for the Euro NCAP reference subset, crosses at 90 degrees.
- The no-response trajectories are synchronized to conflict: the ego front center would contact the crossing vehicle side at 25 percent of its length, matching the Euro NCAP CCCscp impact convention. For non-90-degree exploratory cases, define the conflict zone as the swept intersection of both oriented vehicle footprints.
- Run the learned end-to-end longitudinal policy only in the openpilot mode that exposes and applies that policy. Chill and Experimental behavior must not be pooled because current openpilot releases can use different longitudinal policy paths.
- No safety driver or manual keyboard input is permitted between scenario T0 and termination. A manual intervention is recorded as an intervention outcome and the autonomous run is failed or invalid according to whether it was needed for safety or caused by test-operator error.
- A simulation failure is not automatically a policy-training example. Integration validity, camera timing, model execution, engagement, command propagation, and vehicle response must first pass the attribution checks in this record.
- openpilot is an open-source driver-assistance research system, not an autonomous-driving certification target. The evaluation is a project acceptance test under a stated simulated ODD, not evidence for unsupervised real-world operation.


### Experimental design

#### setup

- **Platform:** Run CARLA 0.9.16 natively on Windows and openpilot under Ubuntu WSL2 using the repository CARLA bridge. Confirm the ScenarioRunner branch matches CARLA 0.9.16 if ScenarioRunner is used.; Use synchronous CARLA mode with fixed_delta_seconds=0.05 s, one and only one tick-owning client, physics substepping compatible with the fixed step, and sensor_tick=0.05 s. The bridge already targets 20 Hz CARLA and camera updates; retain synchronized road and wide-road timestamps.; Use Town10HD_Opt only after selecting and surveying an intersection with straight, lane-centered approach and crossing paths. Add at least two other topologically distinct junctions to regression and training data, even though this item may use one primary controlled junction.
- **Ego and openpilot:** - Spawn the existing ego blueprint vehicle.tesla.model3, record its bounding box and complete physics control, mount the road camera at the repository transform x=0.8 m and z=1.13 m with 40-degree horizontal FOV, and enable the 120-degree wide camera because left-origin crossing hazards can enter outside the narrow camera first.
- Publish roadCameraState and wideRoadCameraState at 20 Hz, CAN/carState/carControl at their configured 100 Hz rates, modelV2 and longitudinalPlan at 20 Hz, and preserve monotonic timestamps. Wait until modeld, controlsd, selfdrived, logging, both cameras, and simulated CAN are healthy before T0.
- Create a dedicated evaluation configuration in which carParams.openpilotLongitudinalControl is true, the intended end-to-end longitudinal policy mode is active, and carControl.actuators.accel is the sole autonomous longitudinal command mapped to CARLA. Retain a separate stock-ACC configuration only as a bridge control experiment.
- Calibrate the acceleration-to-throttle/brake mapping with no-hazard step and ramp tests over the required speed range. Log requested acceleration, applied throttle/brake, measured acceleration, delay, saturation, and stopping distance. Do not infer policy failure if the calibrated actuator cannot realize the request.
- **Crossing actor:** Spawn a representative passenger-car blueprint with known dimensions and stable physics. Test at least a sedan and an SUV or hatchback asset in regression to prevent single-mesh overfitting. Assign a stable actor ID and use deterministic trajectory control rather than Traffic Manager randomness for the core grid.; Accelerate the crossing actor before the evaluated interval, allow at least 0.5 s of constant-speed stabilization, then hold the target speed through the conflict zone. For the Euro NCAP reference subset, synchronize the no-response impact to ego-front-center versus 25 percent along the actor side.; Attach CARLA collision sensing to ego and record world snapshots, transforms, velocities, accelerations, bounding boxes, and control for both actors every simulation tick. Generate instance segmentation and depth only as label/quality channels; do not feed privileged sensors to openpilot.
- **Run sequence:** - Reset the world, set ClearNoon, set dry-road friction and fixed vehicle physics, clear prior control state, then warm up inference and logging while stationary.
- Bring the ego and crossing actor to their prescribed speeds before T0. Define T0 as the first valid frame at the prescribed initial time to conflict with all path, speed, camera, and engagement tolerances satisfied.
- Run until collision, ego stop outside the conflict zone, crossing actor fully clears the ego swept path plus 2 s, ego safely clears the junction, or a fixed timeout. Continue logging for at least 2 s after the safety outcome to measure recovery and false re-acceleration.
- Replay every deterministic cell at least three times and compare frame-level ground truth, camera hashes before dynamic divergence, model outputs, and outcome metrics. Use distinct explicit seeds for stochastic rendering or physics variants.

#### independent_variables

- **Sourced reference grid:** - **Ego speed kmh:** - 20
- 30
- 40
- 50
- 60
- **Crossing vehicle speed kmh:** - 20
- 30
- 40
- 50
- 60
- **Crossing angle deg:** 90
- **Direction:** Left to right in the ego coordinate frame
- **Basis:** Euro NCAP AEB Car-to-Car protocol v4.3.1 CCCscp tests the combinations of VUT and GVT speeds from 20 to 60 km/h in 10 km/h increments, with straight perpendicular paths. Treat this as a benchmark subset, not as certification testing.
- **Project extension grid:** - **Initial time to conflict s:** - 1.0
- 1.5
- 2.0
- 2.5
- 3.0
- 3.5
- 4.0
- 5.0
- **Crossing angle deg:** - 60
- 75
- 90
- 105
- 120
- **Passenger car geometry:** Compact or sedan; SUV or tall hatchback
- **Initial range definition:** Do not sweep range independently from speed and time to conflict in a synchronized constant-speed case. Set ego distance-to-conflict approximately to ego_speed * initial_time_to_conflict and actor distance-to-conflict approximately to actor_speed * initial_time_to_conflict, then solve using oriented bounding-box entry times so both no-response footprints overlap at the selected impact point.
- **Boundary refinement:** After the coarse grid, use bisection or adaptive sampling in 0.1 s time-to-conflict increments around the transition between safe yield and conflict. Sample both sides of the discovered boundary and retain counterfactual actor-absent and actor-time-shifted pairs.
- **Controlled variables:** ClearNoon weather, dry friction, no occluders, no background traffic, no traffic-light intervention, fixed camera exposure/post-processing setting, and no sensor noise for the core deterministic grid; Identical ego route, lane center, vehicle physics, camera calibration, policy mode, model hashes, bridge timing, target speed stabilization, and termination rules; Explicit seed, CARLA frame rate, physics substeps, rendering mode, and CPU/GPU execution configuration
- **Factorial strategy:** Run the full 25-cell Euro NCAP speed grid at 90 degrees and a selected nominal initial time-to-conflict schedule that produces the reference collision geometry. For the wider angle and time-to-conflict design, use a coverage-preserving pairwise or Latin-hypercube design, then exhaustively refine critical boundary cells. Never average left-to-right results with the mirrored direction.

#### observables

- **Carla ground truth:** Frame number, simulation timestamp, transforms, linear/angular velocity and acceleration, oriented bounding boxes, wheel and vehicle physics, traffic controls, friction, and control inputs for ego and crossing actor; Collision events with actor IDs, contact impulse and time; geometric footprint overlap; minimum polygon-to-polygon separation; ego and actor conflict-zone entry and exit times; Actor projected 2D box, pixel area, truncation, camera FOV membership, depth, instance mask, and visible fraction for both road and wide cameras
- **Openpilot inputs and health:** Road and wide road image frame IDs, raw/encoded frame references, camera timestamps, VisionIPC timestamps, calibration transforms, frame age, dropped-frame percentage, and model execution time; can, carState, livePose, liveCalibration, liveDelay, carParams, pandaStates, selfdriveState, controlsState, process health, message validity/aliveness, and engagement events
- **Model and planning:** modelV2 position, velocity, acceleration, action.desiredAcceleration, action.desiredCurvature, action.shouldStop, leads/leadsV3 with probabilities and uncertainty, meta.hardBrakePredicted, confidence class, and raw prediction/model hash where permitted; longitudinalPlan speeds, accelerations, jerks, aTarget, shouldStop, allowBrake, allowThrottle, fcw, source, modelMonoTime, and processing delay
- **Control and vehicle response:** carControl enabled/longActive/latActive state, requested acceleration and steering, carOutput actuator output, applied CARLA throttle/brake/steer, saturation or safety limiting, measured ego longitudinal/lateral acceleration, yaw rate, speed, stopping position, and resume behavior; Monotonic timestamps at sensor capture, bridge receipt, model output, plan output, control output, CARLA application, and measured response so the latency chain can be decomposed
- **Derived events:** First actor visibility, first model response, first braking plan, first brake command, first measured deceleration, maximum braking, ego stop, actor clear, conflict-zone entry/exit, closest approach, collision, manual intervention, and scenario termination

#### expected_safe_behavior

- The system remains engaged and lane-centered, recognizes the developing cross-traffic conflict early enough through its camera-driven policy, and reduces desired and actual longitudinal motion before the ego footprint enters the crossing vehicle's swept path.
- For avoidable cases, ego yields outside the conflict zone, does not collide or geometrically overlap with the crossing vehicle, maintains the project clearance margin, and does not steer into another lane merely to avoid braking.
- For high-urgency but physically avoidable cases, decisive braking may exceed comfort targets but must remain stable, monotonic enough to avoid throttle-brake oscillation, and within the calibrated actuator and friction envelope.
- After the actor fully clears, ego remains stopped or slow until risk has passed, then resumes smoothly without a premature surge. A warning-only response is not sufficient when the project goal is autonomous closed-loop avoidance.
- For a physically unavoidable late trigger, the preferred behavior is prompt maximum feasible mitigation, no destabilizing swerve, and a materially lower impact speed. Such a case is reported as unavoidable/mitigated and is not counted as a positive avoidance pass unless the project acceptance rules explicitly include mitigation.

#### relevant_openpilot_components

- **Input bridge:** tools/sim/bridge/carla/carla_world.py: synchronous 0.05 s CARLA tick, ClearNoon, camera geometry, ego state, camera acquisition, and CARLA actuation; tools/sim/lib/camerad.py and simulated_sensors.py: RGB-to-NV12 conversion, synchronized camera timestamps, VisionIPC, roadCameraState/wideRoadCameraState, IMU/GNSS, and simulated sensor publication; tools/sim/lib/simulated_car.py and bridge/common.py: simulated Honda CAN/panda, engagement, carParams, control selection, longitudinal mode, command conversion, and manual override
- **Inference and policy:** selfdrive/modeld/modeld.py, fill_model_msg.py, parse_model_outputs.py, driving_vision.onnx and driving_policy.onnx: visual and temporal inference, trajectory, desired acceleration/curvature, stopping and lead-related outputs; cereal/log.capnp ModelDataV2: position/velocity/acceleration trajectories, leadsV3, hardBrakePredicted, policy action, confidence and execution metadata
- **Planning and control:** selfdrive/controls/plannerd.py and longitudinal_planner.py: longitudinalPlan generation and source selection; selfdrive/controls/controlsd.py and longcontrol.py: desired actuation, engagement, limiting and carControl publication; cereal services/logging: modelV2 and longitudinalPlan at 20 Hz; carState, controlsState and carControl at 100 Hz; radarState at 20 Hz when applicable
- **Critical configuration finding:** CarlaBridge currently sets alpha_longitudinal_enabled to false and stock_cruise_speed to 8.0 m/s. bridge/common.py then ignores the inactive openpilot acceleration sentinel and runs a bridge-owned speed controller when openpilotLongitudinalControl is false. This must be changed or bypassed in a controlled evaluation branch before closed-loop crossing results can support a policy-retraining decision.


### Evaluation

#### applicable_standards_and_prior_work

- Euro NCAP AEB Car-to-Car Test Protocol v4.3.1, section 8.2.4 CCCscp: straight lane-centered VUT and perpendicular GVT, no-response synchronization to ego front center versus 25 percent of target length, 0.5 s target-speed stabilization, speed combinations from 20 to 60 km/h, path tolerances, termination, and FCW-driver braking procedure. Use as a reference geometry and repeatability protocol only.
- CARLA synchrony and time-step documentation: fixed time steps improve repeatable data collection; synchronous mode lets the client control advancement; only one client should tick; camera/sensor frames must correspond to the same simulation step.
- CARLA sensor reference: sensor data includes frame and timestamp, CARLA uses x-forward/y-right/z-up in the sensor frame, IMU acceleration is m/s^2 and gyroscope output is rad/s. These conventions are required for traceable labels and metrics.
- CARLA ScenarioRunner 0.9.16 and ASAM OpenSCENARIO parameterization: use version matching and externally assigned scenario parameters for reproducible speed, timing, angle and asset sweeps.
- ISO 34502:2022 provides a scenario-based safety evaluation framework during ADS development, but its published scope is limited-access highways. It supports the methodology concept and does not supply intersection pass thresholds for this ADAS test.
- Goff et al., Learning to Drive from a World Model, CVPR Workshops 2025: openpilot's end-to-end policy training is on-policy in reprojective or learned world-model simulation and maps observation/action history to next actions. This motivates closed-loop and counterfactual retraining rather than frame-only object-class fine-tuning.
- The openpilot repository documents software-in-the-loop, replay, panda safety, and hardware-in-the-loop testing, and explicitly characterizes the software as alpha-quality research software. Candidate-policy release gates must retain those existing tests in addition to the CARLA matrix.

#### metrics

- **Safety outcome:** - Collision count and collision severity: count any CARLA collision event or verified oriented-bounding-box overlap; report contact impulse and ego/relative impact speed in m/s.
- Minimum geometric separation d_min in meters: minimum distance between the oriented 2D footprints of ego and actor over all valid ticks; overlap is represented as zero or negative penetration depth in a separate field.
- Conflict-zone occupancy: derive the polygon intersected by the actors' swept paths; record each footprint's entry and exit. Any simultaneous occupancy is a conflict even if CARLA collision callbacks miss contact.
- Post-encroachment time in seconds: when the actor exits before ego enters, PET = t_ego_entry - t_actor_exit. Negative PET denotes overlapping occupancy. Also report the reverse ordering if ego passes first.
- Time to closest approach: with relative position r and velocity v, t_CPA = max(0, -dot(r,v)/dot(v,v)); report predicted distance at CPA and actual closest approach. For crossing geometry also report each actor's time to conflict-zone entry, because one-dimensional following TTC is not sufficient.
- **Response and timing:** Visibility-to-policy latency: first valid reduction in desired acceleration or shouldStop response minus first ground-truth visible frame. Define a response as desiredAcceleration <= -0.5 m/s^2 sustained for at least two 20 Hz model frames, a project measurement threshold rather than a sourced safety threshold.; Policy-to-plan, plan-to-command, command-to-application, and application-to-measured-deceleration latency in milliseconds, each computed from monotonic timestamps. Also report end-to-end visibility-to-measured-deceleration latency.; First-warning time and first-brake time relative to conflict-zone entry, plus time-to-conflict when braking begins. Distinguish modelV2 hardBrakePredicted/FCW from actual autonomous braking.
- **Vehicle dynamics and comfort:** Ego speed profile, stop-line/conflict-zone stopping margin in meters, peak and 95th-percentile longitudinal deceleration in m/s^2, peak lateral acceleration in m/s^2, yaw rate in rad/s, and maximum lane-center deviation in meters.; Longitudinal jerk in m/s^3 from a documented low-pass-filtered acceleration signal; report peak absolute jerk and 95th percentile, filter parameters, and unfiltered trace.; Control stability: count throttle/brake reversals, acceleration sign changes, saturation duration, and premature re-acceleration before actor clearance.
- **Model quality proxies:** Crossing actor lead probability/position error when represented in leadsV3, predicted ego trajectory versus executed trajectory, desired-acceleration error relative to a safety-oracle trajectory, and uncertainty/confidence at decision time.; Because current openpilot is end-to-end and does not expose a complete modular object detector and cross-traffic predictor, absence from leadsV3 alone is not proof of perception failure. Use input ablation, counterfactual pairs, representation probes if available, and policy-action evidence together.
- **Aggregation:** Report every cell, not only global means. Aggregate collision/conflict rate with an exact binomial confidence interval, and continuous metrics with median, 5th/95th percentiles, worst case, and bootstrap confidence intervals grouped by speed, time to conflict, angle, actor asset, intersection and seed.; Use paired comparisons for baseline versus candidate policy on identical scenario manifests. For zero failures in n independent stochastic trials, the one-sided 95 percent upper bound is 1 - 0.05^(1/n); 299 zero-failure trials are required to put this bound below 1 percent. Deterministic duplicate runs verify reproducibility but are not independent safety exposure.

#### pass_criteria

- **Prerequisite validity gates:** openpilot longitudinal control and the intended learned-policy mode are confirmed active from carParams, longitudinalPlan source, carControl and applied CARLA actuation; no bridge-owned stock speed controller controls the safety response.; All required processes and messages remain valid/alive; both camera streams correspond to the correct CARLA tick; no material frame drops, stale timestamps, manual input, safety disengagement, actuator mismatch, or missed simulator tick occurs in the evaluated interval.; Before autonomous intervention, ego and actor satisfy the defined speed and path tolerances. For the Euro NCAP reference subset, use the protocol's lane-path tolerance of 0 +/- 0.05 m and its applicable speed/timing tolerances.
- **Hard safety gates project proposal:** - Zero collision events and zero geometric footprint overlap in every required valid run.
- Ego does not enter the conflict zone before the crossing actor clears; if ego stops, its footprint remains completely outside the crossing path, consistent with the Euro NCAP CCCscp stopping interpretation.
- Minimum geometric separation is at least 0.50 m and PET is at least 1.00 s for avoidable nominal cases. These are conservative project proposals and must be approved against the project's simulated ODD; they are not quoted Euro NCAP thresholds.
- Maximum lane-center deviation is no greater than 0.30 m and the ego remains in its lane. This is a project proposal; a deliberate evasive-steering feature would require a separate scenario and surrounding-lane safety proof.
- No throttle is applied while predicted conflict remains active after braking starts, and no resume begins until the actor footprint clears the ego swept path plus a 0.5 s confirmation window.
- **Comfort and timeliness project proposal:** For initial time to conflict of at least 3.5 s, measured peak deceleration should not exceed 4.0 m/s^2 in magnitude and peak filtered jerk should not exceed 5.0 m/s^3. Treat these as project comfort targets, not hard safety limits for shorter emergencies.; For shorter but physically avoidable cases, safety gates dominate comfort; record and review any deceleration above 6.0 m/s^2 or jerk above 10.0 m/s^3 as a secondary-control or late-response concern.; The response latency distribution and stopping margin must not degrade materially with actor asset or crossing angle; use the worst cell, not only an average.
- **Statistical rule:** Pass the complete 25-cell 90-degree speed reference grid with three deterministic repeats per cell and no hard-gate violation, then pass every defined project-extension acceptance cell with all required stochastic seeds. Invalid-run rate must be below 5 percent per configuration and every invalid run must be rerun after the cause is corrected. If the project claims a stochastic failure probability below 1 percent for a defined stratum, require at least 299 independent zero-failure runs in that stratum or an equivalently justified exact-binomial design. A CARLA pass remains a simulation acceptance result, not certification.
- **Physical unavoidability rule:** Before labeling a late-trigger collision as policy failure, run a ground-truth safety oracle with the same measured friction, actuator delay, acceleration limits and initial state. If even the oracle cannot avoid, classify the cell as physically unavoidable and evaluate mitigation separately. Do not count it as an avoidance pass.

#### fail_criteria

- **Safety failure:** Any collision, geometric overlap, simultaneous conflict-zone occupancy, safety-clearance violation, lane departure, unstable evasive maneuver, or required manual rescue in a valid, oracle-avoidable run; Failure to stop outside the crossing path, premature acceleration before clearance, or a materially higher impact speed than the calibrated safety oracle in an unavoidable case; Any required acceptance cell that repeatedly violates the approved hard gate, even if the overall average passes
- **System or invalid run:** Wrong longitudinal mode, bridge stock-ACC control active, openpilot disengaged, camera or CAN messages stale, camera pair unsynchronized, excessive frame drop, model process lag/crash, CARLA tick mismatch, invalid calibration, actor trajectory outside tolerance, collision sensor inconsistency, or manual operator input not required by the safety protocol; Actor not genuinely visible as specified, target speed not stabilized, no-response geometry not synchronized, wrong crossing direction, wrong map/asset/version, or missing manifest/log channels; An invalid run is excluded from behavioral statistics, its cause is fixed, and the same manifest is rerun. Invalid runs are not silently converted to passes or policy failures.
- **Escalation:** One high-severity valid failure stops candidate release and triggers attribution. It does not by itself authorize retraining. A policy-retraining trigger requires reproducibility, oracle avoidability, valid inputs, correct command application, and evidence that the unsafe choice originated in the learned policy rather than simulation, bridge, planner selection, safety limits, or actuator tracking.

#### failure_attribution

- **Simulator or scenario:** Ground-truth actor path/speed/visibility is wrong, physics is nondeterministic beyond tolerance, collision callbacks disagree with geometry, the target penetrates road geometry, or the oracle outcome changes under identical initial state. Fix the scenario, map or CARLA configuration and rerun.
- **Integration and timing:** Camera pixels, calibration, frame IDs, timestamps, CAN state or coordinate transforms do not match CARLA ground truth; longitudinal mode is false; the bridge stock speed loop is active; or carControl is not the command actually applied. The current stock-ACC bridge behavior falls here by design.
- **Input or visual understanding:** The actor is confirmed visible with adequate pixel area and correct camera delivery, but relevant model outputs/representations do not change against a matched actor-absent counterfactual and the policy remains unsafe. Since openpilot is end-to-end, call this a visual-understanding hypothesis unless a validated object probe demonstrates missed perception.
- **Policy planning:** Inputs and visual sensitivity are valid, the safety oracle can avoid, actuator capacity is sufficient, but modelV2 action/trajectory or the selected longitudinal plan continues into the conflict or brakes too late. Reproduce across at least two independent asset/intersection combinations before treating it as a general policy gap.
- **Planner or mode selection:** modelV2 proposes a safe stop, but longitudinalPlan selects another source, suppresses braking, or the wrong Chill/Experimental path is active. Fix configuration or planner arbitration before training the policy.
- **Control and vehicle:** The policy and plan request adequate deceleration, but carControl, safety limits, actuator conversion, CARLA brake application, or ego dynamics fail to track the request within the calibrated delay and magnitude. Fix controller/plant/bridge mapping.
- **Evidence bundle:** - Scenario manifest and software/model hashes
- CARLA world and sensor ground truth
- Raw road and wide images with frame/timestamp mapping
- modelV2, longitudinalPlan, carControl, carOutput and applied CARLA controls
- Safety-oracle replay and actor-absent/time-shifted counterfactual
- Latency waterfall, metric trace, and deterministic reruns


### Dataset and ML lifecycle

#### dataset_requirements

- **Modalities and rates:** Road and wide-road RGB at the actual 20 Hz policy input cadence, preserving pre-conversion images, exact NV12 buffers or hashes, frame IDs, calibration and monotonic timestamps; CARLA ground truth at every 20 Hz world tick: transforms, kinematics, oriented 3D boxes, actor IDs/classes, lights, controls, collision events, road topology, lane coordinates, conflict-zone geometry, depth and instance masks; openpilot CAN, carState, controlsState, carControl and applied actuators at 100 Hz; modelV2 and longitudinalPlan at 20 Hz; IMU/livePose, process health, engagement/mode and latency signals at native rates
- **Coverage and sampling:** - Cover every ego speed, actor speed, time-to-conflict band, crossing angle, asset geometry and outcome stratum in the designed grid, with extra density within +/-0.5 s of the learned failure boundary.
- Include safe yields, late but avoidable hazards, physically unavoidable mitigation cases, non-conflicting crossings, actor-absent junction passes, and time-shifted counterfactuals so the policy does not learn to brake for every visible side vehicle.
- Balance by risk stratum and scenario rather than by frame. Cap long easy segments, retain pre-event context and at least 2 s post-event recovery, and weight rare high-severity cases without duplicating identical frames.
- Add domain-randomized illumination, texture, camera photometrics, vehicle color/model, small calibration perturbations, friction and actuation delay only after the deterministic clear-visibility core is stable. Complement synthetic data with lawfully collected real crossing and near-crossing clips because CARLA appearance and behavior do not establish sim-to-real validity.
- **Splits and leakage:** Split by complete scenario group, intersection topology/map region, actor asset family, rendered environment seed and source route; never split adjacent frames or counterfactual siblings across train and validation/test.; Freeze a development validation set, an unseen CARLA junction/asset test set, and a real-world complement holdout before mining failures. Keep exact acceptance seeds and all variants derived from them out of training.; Deduplicate using source-run ID, frame ancestry and perceptual hashes. Record parent-child lineage for counterfactuals, crops, relabels and generated variants.
- **Metadata and storage:** Store scenario ID, direction, parameter values, seed, map/OpenDRIVE hash, actor blueprints and dimensions, camera/weather/physics configuration, CARLA/openpilot/bridge/model hashes, policy mode, label version, oracle version, and QA status in a machine-readable manifest.; Use immutable versioned object storage for images/log segments and Parquet or an equivalent typed columnar index for frame/event tables. Hash every artifact, preserve raw logs, and publish a dataset card describing scope, exclusions, synthetic/real proportions, known biases and license/privacy constraints.; A training sample must be traceable back to one raw run and forward to every model trained from it.

#### labeling_and_quality

- **Label schema:** - Per actor and frame: stable ID, passenger-car class and subtype, 3D box, pose, velocity, acceleration, yaw rate, camera projection, depth, occlusion/truncation, visible pixel fraction and in-FOV flags
- Per scenario and frame: ego/actor time to conflict-zone entry, predicted footprint occupancy interval, actual entry/exit, PET, closest approach, separation, collision/impact state, physical avoidability and oracle braking envelope
- Policy supervision: safe trajectory or action distribution, stopping margin, desired acceleration/curvature, should-stop state, conflict probability and recovery target. Preserve multiple valid safe behaviors rather than collapsing all cases to one hard-brake label.
- Outcome taxonomy: safe unchanged pass, comfortable yield, emergency yield, mitigated unavoidable, collision, false-positive braking, premature resume, invalid integration, visual-understanding hypothesis, policy-planning failure, controller failure and simulator failure
- **Ground truth and oracle:** Derive object and geometry labels directly from CARLA actor state and bounding boxes. Compute a constrained safety oracle using the measured ego plant, friction and latency; verify the oracle by replaying its action in CARLA. Treat human review as adjudication of semantic validity and alternative behavior, not as a replacement for precise simulator state.
- **Quality checks:** - Project every 3D box and instance mask into both camera images and automatically reject labels with wrong axis sign, calibration, actor ID or more than one-frame timestamp mismatch.
- Numerically verify speed/range/time-to-conflict consistency, crossing direction, path angle, impact point, conflict polygon, PET and collision against an independent geometry implementation on a sampled set.
- Require complete native-rate streams, no unexpected frame gaps, valid hashes, correct mode/engagement, oracle replay success, and reproducibility within approved tolerance before a sample is eligible for training.
- Double-review all collision, physically unavoidable and failure-attribution labels. Measure reviewer agreement on categorical attribution and resolve disagreements with the complete evidence bundle.
- Represent partial visibility, sensor timing ambiguity and oracle ambiguity explicitly with confidence and exclusion flags. Never substitute an uncertain policy target into the safety-critical training subset.
- **Acceptance gates:** Accept a dataset release only when all required manifests and streams are complete, automated geometry/timing checks pass, zero train-test lineage leaks remain, 100 percent of safety-critical cases receive independent review, sampled 2D/3D projections meet the approved error tolerance, and class/risk coverage matches the dataset card. Any label-generation code change increments the label version and revalidates the frozen audit set.

#### validation_and_regression

- **Offline:** Run identical log/model replay for baseline and candidate with model hashes pinned. Compare desired trajectory, acceleration, shouldStop, hard-brake/lead proxies, latency and counterfactual actor sensitivity on the frozen validation and test sets.; Require no degradation on existing openpilot process replay, model replay, longitudinal maneuver, safety and simulator tests. Inspect calibration and camera preprocessing equivalence before attributing an offline difference to model weights.; Report metrics by time-to-conflict, speed, angle, asset, junction and synthetic/real source with confidence intervals; global improvement cannot hide a worse critical stratum.
- **Closed loop:** Run the candidate on the full deterministic 25-cell reference grid, all extension acceptance cells, boundary stress set, actor-absent false-positive set, mirrored/right-to-left regression, motorcycle regression, occlusion regression and baseline no-hazard suite.; Use paired scenario manifests and independent stochastic seeds. The candidate must have zero new hard-gate failures, must fix the targeted failure family on unseen assets/junctions, and must not increase invalid-run rate.; For every collision or near-boundary difference, replay the safety oracle and produce an attribution bundle before model selection.
- **Release gates:** Zero collision/conflict hard-gate failures in all required valid acceptance runs; no safety regression versus baseline in any frozen stratum; comfort/timing metrics within approved non-inferiority margins; and exact-binomial evidence matching any claimed failure-rate bound; Independent review of dataset lineage, training configuration, model conversion, safety case and rollback package; model and dataset cards updated; all artifacts reproducibly built and signed; Shadow evaluation and then closed-course vehicle testing with trained safety operators, hardware instrumentation and an independent emergency system before any public-road consideration. CARLA success alone is insufficient.
- **Rollback:** Keep the baseline model, bridge, configuration and complete manifest deployable. Automatically block or roll back the candidate on any newly discovered hard safety regression, unexplained mode mismatch, model-conversion discrepancy or production telemetry anomaly. Preserve failed candidate artifacts for analysis rather than overwriting them.


### Decision support

#### risks

- The current bridge does not apply openpilot longitudinal policy; without the prerequisite change, the central research conclusion would be invalid.
- CARLA camera appearance, actor behavior, contact physics, tire/brake model and latency differ from real vehicles; a simulation pass can create false confidence.
- A 40-degree road camera may see a left-origin crossing actor late. The wide camera is necessary, but its preprocessing and model use must match the selected openpilot configuration.
- Crossing traffic is not equivalent to a same-lane lead. leadsV3 may not represent the actor, so naive detection-based attribution can misclassify an end-to-end policy failure.
- Time to conflict, TTC and PET can be implemented incorrectly for crossing swept volumes, especially at non-90-degree angles. Independent geometry validation is required.
- Full factorial sweeps grow quickly and repeated deterministic runs do not provide independent statistical evidence. Adaptive boundary sampling must not replace required acceptance cells.
- Training only on CARLA failures can cause simulator overfitting, indiscriminate braking at intersections, degraded normal driving, or exploitation of rendering artifacts.
- Using the acceptance seeds, adjacent frames or counterfactual siblings in training leaks the test and invalidates improvement claims.
- Very short time-to-conflict cases may be physically unavoidable. Labeling all collisions as policy failures would train unrealistic targets and distort the decision boundary.
- Changing the shipped openpilot policy without a reproducible authorized training/conversion pipeline, functional-safety process and closed-course validation introduces substantial real-world risk.

#### recommendations

- 1. Before adding the crossing actor, create a dedicated evaluation branch/configuration that activates the intended learned longitudinal policy and proves carControl.accel reaches CARLA. Keep the existing stock-ACC path as a negative control and assert the active path in every run.
- 2. Add a deterministic ScenarioRunner/OpenSCENARIO left-to-right CCCscp-style scenario with explicit actor asset, 90-degree geometry, 25-percent side impact synchronization, speed stabilization, seed, parameter manifest and termination criteria.
- 3. Instrument the complete frame-to-actuator latency chain and ground-truth swept-footprint geometry. Add automated validity assertions that abort rather than score a run when camera, mode, path, speed or actuation prerequisites fail.
- 4. Execute the 20-60 km/h by 20-60 km/h reference grid at 90 degrees, then sweep time to conflict and 60-120-degree crossing angles with adaptive refinement around the safety boundary. Use at least sedan and SUV/hatchback targets.
- 5. Adopt the hard safety gates first: no collision, no conflict occupancy, approved separation/PET margin, no lane departure and no premature resume. Keep comfort thresholds explicitly secondary in short emergencies.
- 6. For every failure, automatically generate an actor-absent counterfactual, safety-oracle replay and synchronized attribution report. Never route an invalid or controller-caused failure into the policy dataset.
- 7. Build the versioned event-centric dataset with raw camera, native-rate openpilot logs, CARLA ground truth, lineage-safe splits, unseen junction/asset holdouts and real-world complements. Freeze acceptance cases before training.
- 8. Retrain only on a reproducible, authorized end-to-end/on-policy stack after the stated repeated cross-asset trigger is met. Optimize safe yield and false-positive counterfactual behavior together.
- 9. Gate any candidate through offline replay, full closed-loop crossing and non-crossing regression, existing openpilot safety/process tests, independent review, shadow comparison and closed-course testing, with a signed rollback model ready.
- 10. Report outcomes as CARLA project acceptance within a declared simulated ODD. Do not describe them as Euro NCAP certification or evidence for unsupervised public-road deployment.

#### sources

- **Title:** AEB Car-to-Car Test Protocol, Version 4.3.1
- **Publisher or author:** Euro NCAP
- **Url:** https://cdn.euroncap.com/cars/assets/euro_ncap_aeb_c2c_test_protocol_v431_532926aad1.pdf
- **Publication date:** February 2024
- **Supported claims:** CCCscp geometry, 25-percent side impact synchronization, target stabilization, 20-60 km/h speed matrix, path/speed validity conditions, termination, and the FCW brake-response reference.
- **Title:** Synchrony and time-step
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/
- **Publication date:** Continuously updated documentation; accessed 2026-08-06
- **Supported claims:** Fixed-step data collection, synchronous client-controlled ticking, single tick owner, sensor synchronization, physics substep constraints, and replay considerations.
- **Title:** Sensors reference
- **Publisher or author:** CARLA Simulator documentation
- **Url:** https://carla.readthedocs.io/en/latest/ref_sensors/
- **Publication date:** Continuously updated documentation; accessed 2026-08-06
- **Supported claims:** Sensor frame/timestamp fields, CARLA coordinate conventions, RGB camera data, collision sensing, and IMU units.
- **Title:** ScenarioRunner for CARLA
- **Publisher or author:** CARLA Simulator project
- **Url:** https://github.com/carla-simulator/scenario_runner
- **Publication date:** ScenarioRunner 0.9.16 released 2025-09-29
- **Supported claims:** Scenario definition/execution, CARLA-to-ScenarioRunner version matching, metrics access, and OpenSCENARIO support.
- **Title:** Parameters in ASAM OpenSCENARIO
- **Publisher or author:** ASAM e.V.
- **Url:** https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_DSL/v2.0.0/migration/mg_parameters.html
- **Publication date:** OpenSCENARIO DSL 2.0.0 documentation; accessed 2026-08-06
- **Supported claims:** Externally varied scenario parameters, defaults, distributions and constraints for reproducible parameter sweeps.
- **Title:** ISO 34502:2022 Road vehicles - Test scenarios for automated driving systems - Scenario based safety evaluation framework
- **Publisher or author:** International Organization for Standardization
- **Url:** https://www.iso.org/standard/78951.html
- **Publication date:** 2022-11
- **Supported claims:** Scenario-based evaluation during development and the important limitation that the published framework scope is limited-access highways.
- **Title:** openpilot repository README and Safety and Testing section
- **Publisher or author:** comma.ai
- **Url:** https://github.com/commaai/openpilot
- **Publication date:** Repository accessed 2026-08-06
- **Supported claims:** openpilot system scope, supported testing layers, safety-code role, data logging, and alpha-quality research disclaimer.
- **Title:** Learning to Drive from a World Model
- **Publisher or author:** Mitchell Goff, Greg Hogan, George Hotz, Armand du Parc Locmaria, Kacper Raczy, Harald Schaefer, Adeeb Shihadeh, Weixing Zhang, and Yassine Yousfi; comma.ai
- **Url:** https://openaccess.thecvf.com/content/CVPR2025W/DDADS/papers/Goff_Learning_to_Drive_from_a_World_Model_CVPRW_2025_paper.pdf
- **Publication date:** 2025
- **Supported claims:** End-to-end driving policy formulation, on-policy training using reprojective and learned world-model simulators, closed-loop evaluation, and real ADAS deployment context.
- **Title:** NIST/SEMATECH e-Handbook: Confidence intervals for a proportion
- **Publisher or author:** National Institute of Standards and Technology
- **Url:** https://itl.nist.gov/div898/handbook/prc/section2/prc241.htm
- **Publication date:** Handbook page accessed 2026-08-06
- **Supported claims:** Use of exact binomial confidence intervals when failures or sample sizes are small.
- **Title:** Local CARLA bridge implementation and handoff log
- **Publisher or author:** This openpilot project repository
- **Url:** D:/work/openpilot/tools/sim/carla/IMPLEMENTATION_LOG.md
- **Publication date:** 2026-08-05 to 2026-08-06
- **Supported claims:** Project baseline versions, Windows/WSL architecture, synchronized cameras, fixed 20 Hz CARLA loop, simulated stock ACC, openpilotLongitudinalControl=false behavior, and known actuation configuration constraints.
- **Title:** Local CarlaBridge and common simulator control implementation
- **Publisher or author:** This openpilot project repository
- **Url:** D:/work/openpilot/tools/sim/bridge/carla/carla_bridge.py and D:/work/openpilot/tools/sim/bridge/common.py
- **Publication date:** Inspected 2026-08-06 at repository commit 4df40d2 plus local CARLA integration changes
- **Supported claims:** alpha_longitudinal_enabled=false, stock_cruise_speed=8.0 m/s, branching on carParams.openpilotLongitudinalControl, bridge-owned stock speed loop, and acceleration-to-throttle/brake mapping.
- **Title:** Local openpilot message schema and service frequencies
- **Publisher or author:** comma.ai openpilot repository
- **Url:** D:/work/openpilot/cereal/log.capnp and D:/work/openpilot/cereal/services.py
- **Publication date:** Inspected 2026-08-06 at repository commit 4df40d2
- **Supported claims:** modelV2, longitudinalPlan and carControl fields used for observability and their configured publication/logging frequencies.


## 4. Passenger car crossing right to left with clear visibility

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 5. Motorcycle crossing left to right with clear visibility

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 6. Motorcycle crossing right to left with clear visibility

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 7. Passenger car emerging from occlusion

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 8. Motorcycle emerging from occlusion

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 9. Scenario parameter coverage and automated test generation

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 10. Safety comfort and pass-fail metrics

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 11. Failure attribution and retraining decision

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 12. Dataset specification collection and scenario sampling

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 13. Labeling ground truth and dataset quality assurance

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*

## 14. Policy retraining validation and regression release gates

*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*
