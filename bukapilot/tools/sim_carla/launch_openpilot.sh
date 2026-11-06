#!/bin/bash
# Start bukapilot's manager for a simulator drive (same env as tools/sim/launch_openpilot.sh).
# Started by run_bridge.py --launch-openpilot inside the bukapilot-sim:local container.

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export FINGERPRINT="${FINGERPRINT:-HONDA CIVIC 2016}"

# camerad is replaced by the bridge, loggerd is not needed; add more with BLOCK=...
export BLOCK="${BLOCK:+$BLOCK,}camerad,loggerd"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$( cd "$DIR/../.." && pwd )"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"

cd "$ROOT" && python3 -m tools.sim_carla.lib.params
cd "$ROOT/selfdrive/manager" && exec ./manager.py
