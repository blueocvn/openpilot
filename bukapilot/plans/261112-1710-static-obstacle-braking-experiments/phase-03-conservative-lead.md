# Phase 3: Conservative lead

## Context

For a camera-only lead, `Cluster.get_RadarState_from_vision` (`selfdrive/controls/lib/radar_helpers.py:133`) copies the model's `x` and `v` into `dRel` and `vLead`. `get_lead` (`selfdrive/controls/radard.py:69`) accepts the lead only while `prob > 0.5`, with no memory. Measured: `x` is 5 to 7 m too long at close range, `prob` dips below 0.5 for single frames, and in Town04 `v` stays near ego speed.

Requires user approval before coding: shared code.

## Parts

Each part sits behind its own flag so the benchmark can score them separately.

| Part | Change | Flag | Fails first when |
|---|---|---|---|
| A. Distance margin | `dRel = x - RADAR_TO_CAMERA - margin`, with `margin = clip(K_STD * xStd, MARGIN_MIN, MARGIN_MAX)`. Starting values: `K_STD = 1.0`, `MARGIN_MIN = 3 m`, `MARGIN_MAX = 10 m` | `VisionLeadDistanceMargin` | `xStd` does not track the error on real images |
| B. Closing speed from distance | Low-pass the frame-to-frame change of `dRel` (0.5 s window) while `xStd < 5 m`; `vLead = min(model v, v_ego + filtered closing rate)` | `VisionLeadSpeedFromDistance` | Distance noise is larger than the closing rate: phantom braking |
| C. Lead hold | Keep the last vision lead for up to 1.0 s after `prob` drops below 0.5, extrapolated with its last relative speed; release at once if `prob < 0.1` | `VisionLeadHold` | A vehicle that really left the lane keeps the car braking for 1 s |

Flags are openpilot `Params` booleans read at start. They behave identically in the sim and on the car, and stay off by default until the user decides.

## Files

- Edit `selfdrive/controls/lib/radar_helpers.py`: margin and speed in `get_RadarState_from_vision` (needs `xStd` from `lead_msg`).
- Edit `selfdrive/controls/radard.py`: hold state per lead slot in `RadarD`, flags.
- Edit the fork's params key list (`selfdrive/common/params.cc`): three keys.
- Add `selfdrive/controls/tests/test_vision_lead.py`: unit tests with synthetic lead messages.
- Edit `tools/sim-linux-docker/run.sh`: `--param KEY=1` pass-through so the benchmark can toggle flags. This is generic plumbing, not sim-specific behaviour.

## Steps

1. Offline first, no code change: log `x`, `xStd`, `v`, `prob` per frame with ground truth in the phase 1 scenarios. Compute what margin and what filtered closing rate each part would have produced, and the false-brake rate on the clear-road run. Drop any part that fails here.
2. Present the offline numbers to the user and get approval for the surviving parts.
3. Implement the parts, with tests written first for: margin bounds, speed never above the model's, hold timing and release.
4. Run the short benchmark per part, then the full matrix for the best combination.

## Validation

- Unit tests pass. Existing `selfdrive/controls/tests` still pass.
- Flags off: benchmark equals the phase 1 baseline within its noise floor.
- Flags on: minimum gap behind stationary motorcycle, car and barrier rises to 4 m or more, with no new false braking on the clear road.

## Risk and rollback

Radar-equipped cars are unaffected: the edits are in the vision-only branch. Rollback is turning the flags off; code removal is one revert.
