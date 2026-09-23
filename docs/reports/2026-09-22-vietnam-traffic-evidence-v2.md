# Vietnam traffic Phase 1 / Phase 2 evidence protocol

This protocol is frozen before confirmation runs. Phase 1 is stock Openpilot
with `VN_TRAFFIC_MODE=0`; Phase 2 uses the existing simulation-only policy
with `VN_TRAFFIC_MODE=1`. Neither model weights, planner, bridge nor
motorcycle controller are changed by this protocol.

Run visible 60-second episodes from `openpilot/tools/sim/carla` with Town04,
spawn 40, `motorcycle_weave`, case `alternating`, and `--no-experimental-mode`.
Use seed 42–44 for instrumentation only. Confirmation is seed 45–49, twice
per seed and mode, sequentially. Retain report directories, manifests, rotated
JSONL, planner traces, and user-captured timestamped clips for both cut-in
directions.

Comparable exposures have the same direction and differ by no more than 2 m
at measured entry gap, 1.5 m/s ego speed, and 0.5 s lane hold. At least two
one-to-one matches per direction are required; unmatched difficult events are
reported and invalidate the pair. Contacts remain safety outcomes. Comfort
windows end at first contact and otherwise span one second before entry to two
seconds after exit without double-counting overlapping windows.

`run_valid` requires a finite complete run, >=90% longitudinal active, regular
10 Hz control and 2 Hz ground truth, complete fresh (<250 ms) pose/Openpilot
attribution, complete planner trace and mode-0 replay parity, runtime receipt,
and a derived ONNX-to-runtime build proof. `comparison_valid` additionally
requires matching exposure evidence. `phase2_improved` is only evaluated after
both gates: 15% jerk reduction, no extra switches, >=95% progress, no increased
contact/disengagement, TTC not worse by >0.2 s, and clearance not worse by
>0.25 m. A user must confirm one synchronized clip per direction; otherwise
final visual acceptance remains pending.

Current status: no confirmation result. The isolated rebuild hash does not
match the runtime artifact, so the source-provenance gate remains false. The
honest conclusion until that is resolved is **chưa so sánh được**.
