#!/usr/bin/env bash
set -euo pipefail

readonly OPENPILOT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
readonly STATE_ROOT="$OPENPILOT_ROOT/.carla"
readonly LOG_ROOT="$STATE_ROOT/logs"
export PATH="$STATE_ROOT/bin:$PATH"
export PYTHONUNBUFFERED=1

# tinygrad loads whichever libnvrtc it finds, and a CUDA toolkit newer than the
# display driver produces PTX the driver's JIT rejects with
# CUDA_ERROR_UNSUPPORTED_PTX_VERSION. Pinning a 12.8 nvrtc avoids that: it still
# targets Blackwell (sm_120) but emits PTX every CUDA 12.8+ driver accepts.
# The runtime package alongside it ships the CUDA headers (cuda_fp16.h and
# friends) that nvrtc needs to compile the model's half-precision kernels --
# this box has no full CUDA toolkit installed, only the driver's own runtime
# libs, so nothing else on the system provides them.
readonly NVRTC_PIN="nvidia-cuda-nvrtc-cu12==12.8.93"
readonly CUDA_HEADERS_PIN="nvidia-cuda-runtime-cu12==12.8.90"
readonly NVRTC_DIR="$STATE_ROOT/nvrtc"
readonly NVRTC_LIB="$NVRTC_DIR/nvidia/cuda_nvrtc/lib/libnvrtc.so.12"
readonly CUDA_HEADERS_DIR="$NVRTC_DIR/nvidia/cuda_runtime"

use_pinned_nvrtc() {
  [[ -f "$NVRTC_LIB" ]] || return 0
  # tinygrad reads NVRTC_PATH first; LD_LIBRARY_PATH covers libnvrtc-builtins.
  export NVRTC_PATH="$NVRTC_LIB"
  export LD_LIBRARY_PATH="${NVRTC_LIB%/*}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  [[ -d "$CUDA_HEADERS_DIR/include" ]] && export CUDA_PATH="$CUDA_HEADERS_DIR"
}
use_pinned_nvrtc

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

cuda_runs_a_kernel() {
  # Opening the device is not enough: compilation and module load are where a
  # driver/toolkit mismatch actually fails, and float32 alone does not catch a
  # missing cuda_fp16.h -- the model's kernels are half-precision. Compile and
  # run one real kernel of each to know the actual build will work.
  uv run --extra carla --extra tools python - <<'PY' >/dev/null 2>&1
from tinygrad import Tensor, dtypes
assert Tensor.ones(16, device="CUDA").contiguous().sum().item() == 16.0
assert Tensor.ones(16, device="CUDA", dtype=dtypes.float16).contiguous().sum().item() == 16.0
PY
}

select_modeld_device() {
  # The driving model paces the whole control loop. openpilot's default PC
  # build compiles it for CPU, where it manages ~2 Hz against the 20 Hz
  # controlsd expects, so the car steers on a half-second-old plan and weaves.
  # CARLA already requires a GPU, so build the model for it when one works.
  # Set MODELD_DEV explicitly to override, including MODELD_DEV=CPU.
  if [[ -n "${MODELD_DEV:-}" ]]; then
    echo "modeld: building for $MODELD_DEV (MODELD_DEV set)"
    return
  fi
  if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi -L >/dev/null 2>&1; then
    echo "modeld: no NVIDIA GPU visible, using the CPU build" >&2
    echo "modeld: the model will not hold 20 Hz; expect the car to weave" >&2
    return
  fi
  if [[ ! -f "$NVRTC_LIB" ]] || [[ ! -d "$CUDA_HEADERS_DIR/include" ]]; then
    echo "modeld: installing pinned nvrtc + CUDA headers ($NVRTC_PIN, $CUDA_HEADERS_PIN)"
    uv pip install --target "$NVRTC_DIR" "$NVRTC_PIN" "$CUDA_HEADERS_PIN"
    use_pinned_nvrtc
    # tinygrad's kernel cache keys on source+arch, not on which nvrtc produced
    # it. A prior attempt against the system CUDA toolkit (13.3, PTX the
    # driver's JIT rejects) can leave broken entries that this pin would
    # otherwise keep serving forever. Clear both cache locations this repo has
    # used (default and run.sh's own XDG_CACHE_HOME) since this is the one
    # point where we know the nvrtc in use just changed.
    rm -f "${XDG_CACHE_HOME:-$HOME/.cache}/tinygrad/cache.db"{,-wal,-shm}
    rm -f "$HOME/.cache/tinygrad/cache.db"{,-wal,-shm}
  fi
  if cuda_runs_a_kernel; then
    export MODELD_DEV=CUDA
    echo "modeld: CUDA compiles and runs, building the model for it"
  else
    echo "modeld: a GPU is present but tinygrad could not run a CUDA kernel;" >&2
    echo "modeld: falling back to the CPU build, which will not hold 20 Hz." >&2
    echo "modeld: to see the error, run:" >&2
    echo "modeld:   uv run --extra carla --extra tools python -c 'from tinygrad import Tensor; Tensor.ones(16, device=\"CUDA\").contiguous().sum().item()'" >&2
  fi
}

setup() {
  sudo apt-get update
  sudo apt-get install -y git-lfs curl clang build-essential
  git lfs install --local --force
  git lfs pull
  if ! command -v uv >/dev/null 2>&1; then
    export UV_INSTALL_DIR="$STATE_ROOT/bin"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$UV_INSTALL_DIR:$PATH"
  fi
  ./tools/op.sh setup
  uv sync --extra carla --extra tools
  select_modeld_device
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
