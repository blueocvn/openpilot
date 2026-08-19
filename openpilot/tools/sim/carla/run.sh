#!/usr/bin/env bash
set -euo pipefail

readonly OPENPILOT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
readonly STATE_ROOT="$OPENPILOT_ROOT/.carla"
readonly LOG_ROOT="$STATE_ROOT/logs"
export PATH="$STATE_ROOT/bin:$PATH"
export PYTHONUNBUFFERED=1

if [[ ! -f "$OPENPILOT_ROOT/pyproject.toml" ]]; then
  echo "Expected openpilot at $OPENPILOT_ROOT; WSL must use D:\\work\\openpilot." >&2
  exit 1
fi

cd "$OPENPILOT_ROOT"
# Git dependency caches need Linux filesystem semantics. Ubuntu's VHD is stored at
# D:\wsl\ubuntu, so these still consume space on D: rather than the Windows C: drive.
readonly WSL_CACHE_ROOT="$HOME/.cache/openpilot-carla"
mkdir -p "$WSL_CACHE_ROOT/uv" "$WSL_CACHE_ROOT/tmp" "$LOG_ROOT"
export UV_CACHE_DIR="$WSL_CACHE_ROOT/uv"
export XDG_CACHE_HOME="$WSL_CACHE_ROOT"
export TMPDIR="$WSL_CACHE_ROOT/tmp"
export CARLA_HOST="${CARLA_HOST:-$(awk '$2 == "00000000" {g=$3; printf "%d.%d.%d.%d\n", strtonum("0x" substr(g,7,2)), strtonum("0x" substr(g,5,2)), strtonum("0x" substr(g,3,2)), strtonum("0x" substr(g,1,2)); exit}' /proc/net/route)}"
export CARLA_PORT="${CARLA_PORT:-2000}"
# The bridge and manager run in separate processes. Keep their fingerprint
# consistent with the CARLA Tesla rather than inheriting a test-car default.
export FINGERPRINT="TESLA_MODEL_3"
export SIMULATOR="carla"
export SCALE="${SCALE:-3.0}"
export BLOCK="${BLOCK:-},soundd"
# Visible CARLA runs should also show the openpilot UI by default. Set
# OPENPILOT_UI=0 explicitly for headless/integration runs.
if [[ "${OPENPILOT_UI:-1}" == "1" ]]; then
  unset CI
else
  export CI=1
fi

windows_launcher() {
  local script_path
  script_path="$(wslpath -w "$OPENPILOT_ROOT/openpilot/tools/sim/carla/run.ps1")"
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$script_path" "$@"
}

start_server() {
  if [[ "${CARLA_HEADLESS:-0}" == "1" ]]; then
    windows_launcher start "$@"
  else
    windows_launcher start -Visible "$@"
  fi
}

build_native_converter() {
  local target="openpilot/tools/sim/lib/libyuv_pyx.so"
  # The binary is intentionally not versioned. Build it on first use and after
  # its Cython source changes, while avoiding a full SCons startup on every
  # simulator launch.
  if [[ ! -f "$target" || openpilot/tools/sim/lib/libyuv_pyx.pyx -nt "$target" ]]; then
    uv run --extra carla --extra tools scons "$target"
  fi
}

setup() {
  sudo apt-get update
  sudo apt-get install -y git-lfs curl clang build-essential
  git lfs install --local
  git lfs pull
  if ! command -v uv >/dev/null 2>&1; then
    export UV_INSTALL_DIR="$STATE_ROOT/bin"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$UV_INSTALL_DIR:$PATH"
  fi
  ./tools/op.sh setup
  uv sync --extra carla --extra tools
  # Current openpilot imports several generated modules as soon as manager
  # starts (rednose, longitudinal MPC, UI input metadata, and bootlog). Build
  # them during setup so a fresh clone is ready for its first simulator run.
  uv run --extra carla --extra tools scons --minimal -j4
  windows_launcher setup
}

start_bridge() {
  build_native_converter
  uv run --extra carla --extra tools openpilot/tools/sim/run_bridge.py \
    --simulator carla --carla-host "$CARLA_HOST" --carla-port "$CARLA_PORT" "$@" \
    2>&1 | tee "$LOG_ROOT/bridge.log"
  return "${PIPESTATUS[0]}"
}

stop_openpilot() {
  # A terminal can disappear without removing its pid file, and manager/bridge
  # children may create their own sessions. Clean only processes rooted in this
  # openpilot simulation, first gracefully and then forcibly if necessary.
  pkill -TERM -f "$OPENPILOT_ROOT/openpilot/system/manager/manager.py" 2>/dev/null || true
  pkill -TERM -f 'openpilot/tools/sim/run_bridge.py --simulator carla' 2>/dev/null || true
  # Managed processes replace argv[0] with module names (for example
  # selfdrive.modeld.modeld), so they no longer contain OPENPILOT_ROOT after
  # their manager exits. Match only openpilot's namespaced process titles.
  pkill -TERM -f '^(selfdrive|system)\.' 2>/dev/null || true
  sleep 2
  pkill -KILL -f "$OPENPILOT_ROOT/openpilot/system/manager/manager.py" 2>/dev/null || true
  pkill -KILL -f 'openpilot/tools/sim/run_bridge.py --simulator carla' 2>/dev/null || true
  pkill -KILL -f '^(selfdrive|system)\.' 2>/dev/null || true
}

case "${1:-all}" in
  setup) setup ;;
  server) shift; start_server "$@" ;;
  bridge) shift; start_bridge "$@" ;;
  all)
    # Forward any extra flags to the bridge so the map and spawn point can be
    # chosen per run, e.g. --carla-town Town04_Opt --carla-spawn-point 40.
    shift || true
    start_server
    export OPENPILOT_MANAGER_LOG="$LOG_ROOT/openpilot.log"
    start_bridge --launch-openpilot "$@"
    ;;
  status)
    windows_launcher status || true
    if pgrep -f "$OPENPILOT_ROOT/openpilot/system/manager/manager.py" >/dev/null; then
      echo "openpilot manager is running"
    else
      echo "openpilot manager is not running"
    fi
    if pgrep -f 'openpilot/tools/sim/run_bridge.py --simulator carla' >/dev/null; then
      echo "CARLA bridge is running"
    else
      echo "CARLA bridge is not running"
    fi
    ;;
  stop)
    stop_openpilot
    windows_launcher stop
    ;;
  *)
    echo "Usage: $0 {setup|server|bridge|all|status|stop} [bridge args...]" >&2
    echo "  highway run: $0 all --carla-town Town04_Opt --carla-spawn-point 40" >&2
    exit 2
    ;;
esac
