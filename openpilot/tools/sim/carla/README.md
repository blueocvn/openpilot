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
active and ego reaches 7.5 m/s. It maintains 3–5 background Vespas in the two
adjacent lanes at 5–7 m/s. One additional motorcycle at a time cuts across
the ego lane: seeded attempts every 2–3 s, 4.5–5.5 m/s speed, 6–8 m planned
bumper gap on entering the ego lane, and a 1.5–2 s lane hold. A new attempt
waits while the previous crossing is active. The scene checks vehicle
footprints at spawn and defers on occupied lanes or insufficient road; it does
not guarantee collision avoidance after a close cut-in. Actors are retired
after crossing or leaving the available road; none are teleported in front of
ego. This intentionally replaces the previous lower-speed baseline.

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
cut-ins with lead or real brake actuation, stop/go, clearance, and collisions:

```bash
uv run python ../analyze_motorcycle_weave.py \
  ../../../../.carla/reports/motorcycle-weave-YYYYMMDD-HHMMSS-NNNNNNNNN
```
