# CARLA handoff source manifest

> **Historical record.** This manifest describes a handoff archive built against
> the pre-restructure layout, where `selfdrive/`, `system/` and `tools/` sat at
> the repository root. On the current tree that source lives under `openpilot/`
> (for example `openpilot/tools/sim/lib/camerad.py`). Paths below are left as
> shipped so the archive contents stay verifiable; do not follow them as
> instructions for this checkout.

This package is a source overlay for openpilot `v0.11.1` at commit `4df40d2`.
It does not contain the complete openpilot repository or the CARLA runtime.

## Apply the overlay

1. Clone openpilot and check out commit `4df40d2`.
2. Initialize its Git submodules and Git LFS objects.
3. Extract this archive into the repository root, preserving paths and replacing
   matching files.
4. Restore executable permissions if the ZIP extractor removed them:

   ```bash
   chmod +x tools/sim/carla/run.sh tools/sim/launch_openpilot.sh tools/sim/run_bridge.py
   ```

5. Follow `tools/sim/carla/installation.md`.

## Included source

- `.gitignore`
- `pyproject.toml`
- `uv.lock`
- `selfdrive/modeld/modeld.py`
- `tools/sim/bridge/common.py`
- `tools/sim/bridge/carla/`
- `tools/sim/carla/`
- `tools/sim/launch_openpilot.sh`
- `tools/sim/lib/camerad.py`
- `tools/sim/lib/common.py`
- `tools/sim/lib/simulated_car.py`
- `tools/sim/lib/simulated_sensors.py`
- `tools/sim/run_bridge.py`
- `tools/sim/tests/test_carla_bridge.py`

## Deliberately excluded

- `.git/` and the rest of the upstream openpilot source
- `.carla/` downloads, CARLA binaries, profiles, caches, and logs
- `.venv/`, Python caches, build products, and model caches
- WSL virtual disks and legacy simulator state

The archive root also contains `SHA256SUMS.txt` so recipients can verify every
included file after extraction.
