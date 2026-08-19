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

Optional openpilot longitudinal control:

```bash
bash run.sh all --openpilot-longitudinal \
  --carla-town Town10HD_Opt --carla-spawn-point 16
```
