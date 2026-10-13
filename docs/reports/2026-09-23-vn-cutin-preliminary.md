# Phase 2 cut-in — preliminary diagnostic, not acceptance

Date: 2026-09-23. Branch: `feat/motorcycle`. Scope: visible CARLA/WSL only;
no model, compiled artifact, radard, bridge, or real-car changes.

## Result

**Collision avoidance not achieved.** A final-code visible Enhancement run used
`VN_TRAFFIC_MODE=1`, `close_cut_in`, `right_to_left`, seed 42, Town04_Opt
spawn 40, 30 seconds, experimental mode off. Report:
`.carla/reports/motorcycle-weave-f9448c38fe444b4a9a423a184932f6ab`.
The cut-in started at scene 3.20 s, entered the ego lane at 4.70 s with a
3.20 m footprint gap, and contacted ego at 5.80 s. Planner trace was complete
(834 cycles, no sequence gaps). The new predictor never confirmed a cut-in;
MPC never selected `lead2`. The first negative planner request was about
0.151 s before entry. Minimum requested acceleration was -3.5 m/s²; measured
CARLA acceleration reached -3.00 m/s² before contact and ego speed fell.
Maximum normalized CARLA brake was 0.049, but that number alone does not
prove a bridge fault. Lateral tracking P95 was 0.766 m (>0.4 m gate), though
the maximum was below the 0.8 m ceiling. This is a **failed, invalid-gate
diagnostic**, not an accepted confirmation sample. A synchronized user clip
was unavailable.

An earlier, during-development visible Enhancement run used
`VN_TRAFFIC_MODE=1`, `close_cut_in`, `right_to_left`, seed 42, Town04_Opt
spawn 40, 30 seconds, experimental mode off. Report:
`.carla/reports/motorcycle-weave-457ee4eb336440b0a8a87881ea0811f9`.
The cut-in started at scene 3.20 s, entered the ego lane at 4.75 s with a
3.00 m footprint gap, and contacted ego at 5.65 s. Planner trace was complete
(811 cycles, no sequence gaps). The new predictor never confirmed a cut-in;
MPC never selected `lead2`. The first negative planner request was about
0.142 s before entry, much too late to prevent this contact. Minimum requested
acceleration was -3.5 m/s²; measured CARLA acceleration reached -1.62 m/s²
before contact and ego speed fell. A small normalized CARLA brake value
(maximum 0.041) alone does not prove a bridge fault.

That earlier scene run also failed the stipulated tracking gate: lateral P95
0.812 m > 0.4 m and maximum 0.812 m > 0.8 m. Policy geometry was still
being finalized during that run, so its manifest is not a frozen-policy
receipt either.

## Same-input replay and model limitation

The Original seed-42 alternating trace
`.carla/reports/motorcycle-weave-4b3756e4c96747598fb5445a45952476`
contains 774 complete planner inputs. Mode-0 replay against **both** planner
and MPC from pre-change commit `6a12538` gave equal MPC parameters, solver
states, trajectories, plan source and output acceleration for all 774 ticks.
Running current mode 1 on these same inputs produced zero confirmed cut-in
threats and zero `lead2` plans before entry. The first negative output was
approximately 0.009 s before entry in the replay.

In that Original trace, a model lead hypothesis first exceeded probability
0.55 about 0.61 s before actual lane entry. Its predicted path appeared to
cross the ego corridor, but its lateral uncertainty at the estimated crossing
exceeded the fixed 1.5 m gate; later predictions no longer represented a
*future* entry. The predictor therefore rejected it. No CARLA actor position
or schedule was fed back as a synthetic lead. An archived `left_to_right`
trace likewise gave no confirmed cut-in. Its high-probability lead hypothesis
had positive lateral model coordinates while the active actor's measured
lateral offset was negative, so actor identity cannot be inferred from lead
probability or timing alone. The analyzer labels matching-side evidence as
side-compatible, not actor-proven, and refuses opposite-side attribution.

## Gate status and next decision

### Follow-up replay after frame tracking

The recorded final-code run above was replayed after adding hypothesis-local
error handling and frame-to-frame lateral tracking. This is a counterfactual
planner replay, not a new CARLA result. The updated predictor first confirmed
`frame_motion` 0.316 s before lane entry and retained/reacquired the track
through short uncertainty. Mode 0 remained equal to the pre-enhancement
planner and MPC over all 774 reference ticks.

MPC still did not report `lead2`. The first negative planner output remained
about 0.153 s before entry, although it became more negative on that frame
(approximately -0.248 instead of -0.167 m/s² in the replay). The model lead
was estimated around 16–18 m away as the measured actor gap approached the
3.20 m entry gap. The predictor therefore improves the timing of the warning
signal but has not yet demonstrated enough longitudinal accuracy or control
effect to avoid contact. The archived 10-second steady-lead control produced
no cut-in confirmation.

A new visible Enhancement run used the final tracking and startup-ramp code:
`.carla/reports/motorcycle-weave-51346c6262cf4430900625e397dee576`.
After engagement, terminal telemetry began at throttle 0.06 and increased
without the previous full-throttle/brake alternation. This is positive evidence
for smoother startup, though the scene report begins only after ego reaches
7.5 m/s and therefore does not independently score the launch interval.

The new run still contacted the motorcycle. Entry gap was 2.97 m, entry TTC
0.739 s, contact occurred at scene time 5.60 s, and lateral tracking P95 was
0.754 m, so the scene gate remained invalid. The live predictor did not
confirm the cut-in and MPC did not select `lead2`; the first negative planner
request was 0.171 s before entry. This run therefore does not validate
collision avoidance, and it shows that the replay improvement is not stable
across repeated model observations of the same seeded scene.

The model-signal and tracking development gates have failed on the current
evidence. A 10-second visible `steady_lead` smoke test
(`.carla/reports/motorcycle-weave-c30b6cbc7899469fbaaa17b3d8e2b0f3`)
spawned one same-lane actor, no cut-in or contact, and no predictor
confirmation. It is not a full-duration control comparison. Confirmation
seeds 45–49, two repeats per mode/direction, and the full 30-second control
set were **not** run; there is no Phase-2 improvement claim. The next
technical decision is to inspect whether a different
*model-observed* signal can identify a crossing actor early and consistently
without false braking. Any change to the fixed uncertainty/confirmation
thresholds, scene geometry, model, or detector requires a separate reviewed
plan and new validation; it cannot be called acceptance of this one.
