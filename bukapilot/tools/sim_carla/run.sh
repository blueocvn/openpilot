#!/usr/bin/env bash
# bukapilot + CARLA on Windows/WSL2, no Docker. Structure follows
# openpilot/tools/sim/carla/run.sh:
#   * CARLA 0.9.12 runs natively on Windows (started through run.ps1 via powershell.exe)
#   * bukapilot, the bridge and its UI run in the Ubuntu WSL distro, in a Python 3.8 venv
#
#   bash run.sh setup                                   # one time: packages, venv, libyuv, scons, CARLA
#   bash run.sh all [bridge args]                       # CARLA + bukapilot + bridge
#   bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16
#   bash run.sh server | bridge [args] | status | stop
#
# Environment: CARLA_HEADLESS=1, CARLA_QUALITY=Low|High, CARLA_PORT=2000, CARLA_HOST=<ip>,
#              CARLA_ROOT=/mnt/e/CARLA/0.9.12/WindowsNoEditor (reuse an existing install),
#              WSL_DISTRO=Ubuntu-22.04, OP_UI=0|1, SKIP_BUILD=1
set -euo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SIM_DIR/../.." && pwd)"            # bukapilot root
STATE_ROOT="$ROOT/.carla"
LOG_ROOT="$STATE_ROOT/logs"
mkdir -p "$LOG_ROOT"

# Python venv and caches live on the Linux filesystem (drvfs is slow and has no symlinks).
WSL_CACHE_ROOT="$HOME/.cache/bukapilot-carla"
VENV="$WSL_CACHE_ROOT/venv"
mkdir -p "$WSL_CACHE_ROOT/uv" "$WSL_CACHE_ROOT/tmp"
export UV_CACHE_DIR="$WSL_CACHE_ROOT/uv"
export TMPDIR="$WSL_CACHE_ROOT/tmp"
export PATH="$STATE_ROOT/bin:$HOME/.local/bin:$VENV/bin:$PATH"
export PYTHONUNBUFFERED=1
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"

WSL_DISTRO="${WSL_DISTRO:-${WSL_DISTRO_NAME:-Ubuntu-22.04}}"
# The Windows host is the WSL2 default gateway.
default_gateway() { ip route show default 2>/dev/null | awk '{print $3; exit}'; }
export CARLA_HOST="${CARLA_HOST:-$(default_gateway)}"
export CARLA_PORT="${CARLA_PORT:-2000}"
# bukapilot's own Qt UI opens through WSLg; OP_UI=1 selects openpilot's host UI (op_ui/host_ui.py).
export OP_UI="${OP_UI:-0}"

windows_launcher() {
  local script_path extra=()
  script_path="$(wslpath -w "$SIM_DIR/run.ps1")"
  [ -n "${CARLA_ROOT:-}" ] && extra+=(-CarlaRoot "$(wslpath -w "$CARLA_ROOT")")
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$script_path" \
    -Port "$CARLA_PORT" -Quality "${CARLA_QUALITY:-Low}" -WslDistro "$WSL_DISTRO" "${extra[@]}" "$@"
}

start_server() {
  if [[ "${CARLA_HEADLESS:-0}" == "1" ]]; then
    windows_launcher -Command start "$@"
  else
    windows_launcher -Command start -Visible "$@"
  fi
}

# libyuv headers are vendored in third_party/libyuv/include, but the static library is not
# (*.a is git-ignored). Build the commit recorded in third_party/libyuv/build.txt.
build_libyuv() {
  local target="$ROOT/third_party/libyuv/x64/lib/libyuv.a"
  [ -f "$target" ] && return 0
  local src="$WSL_CACHE_ROOT/libyuv-src" commit
  commit="$(awk '/git reset --hard/{print $4}' "$ROOT/third_party/libyuv/build.txt")"
  [ -d "$src/.git" ] || git clone https://chromium.googlesource.com/libyuv/libyuv "$src"
  git -C "$src" reset --hard "$commit"
  (cd "$src" && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release . && make -j"$(nproc)" yuv)
  mkdir -p "$(dirname "$target")"
  cp "$src/libyuv.a" "$target"
  echo "built $target"
}

# acados' x86_64 shared libraries are git-ignored (*.so), so only libqpOASES_e.so.3.1 is in this
# tree. Fetch the prebuilt ones from the upstream bukapilot repo (Ubuntu 20.04 builds; they run on 22.04).
fetch_acados_libs() {
  local dir="$ROOT/third_party/acados/x86_64/lib" base="https://raw.githubusercontent.com/kommuai/bukapilot/HEAD/third_party/acados/x86_64/lib" f
  mkdir -p "$dir"
  for f in libacados.so libblasfeo.so libhpipm.so; do
    [ -s "$dir/$f" ] || curl -fL --retry 3 -o "$dir/$f" "$base/$f"
  done
  # upstream stores libqpOASES_e.so as a symlink, which is a text file on Windows checkouts
  [ -s "$dir/libqpOASES_e.so" ] && [ "$(stat -c %s "$dir/libqpOASES_e.so")" -gt 1000 ] || cp "$dir/libqpOASES_e.so.3.1" "$dir/libqpOASES_e.so"
}

# Python 3.8 deps pinned by Pipfile.lock, same recipe as the old tools/sim/Dockerfile.bukapilot.
# onnxruntime-gpu==1.10.0 is gone from PyPI, so the CPU build of the same version replaces it.
install_python_env() {
  command -v uv >/dev/null 2>&1 || { curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }
  uv python install 3.8
  [ -x "$VENV/bin/python" ] || uv venv --python 3.8 "$VENV"
  python3 - "$ROOT/Pipfile.lock" "$WSL_CACHE_ROOT/requirements.txt" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))['default']
with open(sys.argv[2], 'w') as f:
  for k, v in d.items():
    if k == 'onnxruntime-gpu' or 'version' not in v:
      continue
    f.write(f"{k}{v['version']}" + (f"; {v['markers']}" if 'markers' in v else '') + "\n")
PY
  VIRTUAL_ENV="$VENV" uv pip install -r "$WSL_CACHE_ROOT/requirements.txt" onnxruntime==1.10.0 carla==0.9.12
}

build_bukapilot() {
  [ -n "${SKIP_BUILD:-}" ] && return 0
  fetch_acados_libs
  (cd "$ROOT" && scons -j"$(nproc)")
}

setup() {
  # qt5-default and libavresample-dev no longer exist on Ubuntu 22.04 (qtbase5-dev replaces the first).
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    autoconf build-essential ca-certificates clang cmake make cppcheck libtool \
    gcc-arm-none-eabi bzip2 liblzma-dev libarchive-dev libbz2-dev capnproto libcapnp-dev \
    curl libcurl4-openssl-dev git git-lfs ffmpeg libavformat-dev libavcodec-dev \
    libavdevice-dev libavutil-dev libavfilter-dev libeigen3-dev libffi-dev \
    libglew-dev libgles2-mesa-dev libglfw3-dev libglib2.0-0 libomp-dev libopencv-dev \
    libpng16-16 libssl-dev libsqlite3-dev libusb-1.0-0-dev libzmq3-dev libsystemd-dev \
    locales opencl-headers ocl-icd-libopencl1 ocl-icd-opencl-dev clinfo \
    python3 python3-dev python3-pip python3-distutils \
    qml-module-qtquick2 qtbase5-dev qtmultimedia5-dev qtlocation5-dev qtpositioning5-dev \
    libqt5sql5-sqlite libqt5svg5-dev libqt5x11extras5-dev \
    libreadline-dev libdw1 valgrind tmux dbus libx11-6 pocl-opencl-icd
  install_python_env
  build_libyuv
  build_bukapilot
  windows_launcher -Command setup
}

start_bridge() {
  build_libyuv
  build_bukapilot
  (cd "$SIM_DIR" && python3 ./run_bridge.py "$@") 2>&1 | tee "$LOG_ROOT/bridge.log"
  return "${PIPESTATUS[0]}"
}

stop_bukapilot() {
  pkill -TERM -f "$ROOT/selfdrive/manager/manager.py" 2>/dev/null || true
  pkill -TERM -f 'tools/sim_carla/run_bridge.py' 2>/dev/null || true
  sleep 2
  pkill -KILL -f "$ROOT/selfdrive/manager/manager.py" 2>/dev/null || true
  pkill -KILL -f 'tools/sim_carla/run_bridge.py' 2>/dev/null || true
}

case "${1:-all}" in
  setup) setup ;;
  server) shift; start_server "$@" ;;
  bridge) shift; start_bridge "$@" ;;
  all)
    shift || true
    start_server
    export OPENPILOT_MANAGER_LOG="$LOG_ROOT/bukapilot.log"
    start_bridge --launch-openpilot "$@"
    ;;
  status)
    windows_launcher -Command status || true
    pgrep -f "$ROOT/selfdrive/manager/manager.py" >/dev/null && echo "bukapilot manager is running" || echo "bukapilot manager is not running"
    pgrep -f 'tools/sim_carla/run_bridge.py' >/dev/null && echo "CARLA bridge is running" || echo "CARLA bridge is not running"
    ;;
  stop)
    stop_bukapilot
    windows_launcher -Command stop
    ;;
  *)
    echo "Usage: $0 {setup|server|bridge|all|status|stop} [bridge args...]" >&2
    echo "  example: $0 all --carla-town Town04_Opt --carla-spawn-point 16" >&2
    exit 2
    ;;
esac
