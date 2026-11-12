#!/usr/bin/env bash
# One entry point for bukapilot + CARLA on native Ubuntu (Docker). Same commands as ../sim/run.sh.
#
#   bash run.sh setup                       # one time: build the images (bukapilot-sim:local, carla-sim:0.9.12)
#   bash run.sh setup-ui                    # one time: openpilot's UI venv (.venv + scons --minimal), for OP_UI=1
#   bash run.sh all [bridge args]           # CARLA (background) + bukapilot + bridge
#   bash run.sh all --carla-town Town04_Opt --carla-spawn-point 16
#   bash run.sh all --yellow-path           # openpilot UI (OP_UI=1) draws the driving path in yellow
#   bash run.sh server | bridge [args] | status | stop
#
# Environment: CARLA_HEADLESS=1, CARLA_QUALITY=Low|High (default High), CARLA_PORT=2000,
#              OP_UI=1 (openpilot UI on the host) | 0 (bukapilot Qt UI). Default: 1 if <openpilot>/.venv exists, else 0.
#              SKIP_BUILD=1 (skip scons), SHELL_ONLY=1, YELLOW_PATH=1 (same as --yellow-path; OP_UI=1 only).
#              ALC_MIN_SPEED_KPH=30 (lowest speed for a lane change; real cars keep 30 mph),
#              CARLA_KEYS=0 (no keys from the CARLA window), CARLA_WINDOW_TITLE (default CarlaUE4,=UI).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"          # bukapilot root
OP_ROOT="$(cd "$ROOT/.." && pwd)"         # openpilot repo root (holds .venv)
IMAGE="${IMAGE:-bukapilot-sim:local}"
CARLA_NAME="${CARLA_NAME:-carla_server}"
BRIDGE_NAME="${NAME:-bukapilot_sim_carla}"
export CARLA_PORT="${CARLA_PORT:-2000}"
export CARLA_QUALITY="${CARLA_QUALITY:-High}"
mkdir -p "$ROOT/.carla/logs"

if [ -z "${OP_UI:-}" ]; then
  if [ -x "$OP_ROOT/.venv/bin/python" ]; then OP_UI=1; else OP_UI=0; fi
fi
if [ "$OP_UI" == "1" ] && [ ! -x "$OP_ROOT/.venv/bin/python" ]; then
  echo "openpilot venv not found at $OP_ROOT/.venv -> using bukapilot's Qt UI (OP_UI=0)." >&2
  echo "  to get openpilot's UI: bash run.sh setup-ui" >&2
  OP_UI=0
fi
export OP_UI

setup() {
  docker image inspect "$IMAGE" >/dev/null 2>&1 || (cd "$ROOT" && docker build -t "$IMAGE" -f tools/sim-linux-docker/Dockerfile.bukapilot .)
  # start_carla.sh builds carla-sim:0.9.12 on first use (windowed) or pulls carlasim/carla:0.9.12 (headless)
  if [ "${CARLA_HEADLESS:-0}" == "1" ]; then
    docker image inspect carlasim/carla:0.9.12 >/dev/null 2>&1 || docker pull carlasim/carla:0.9.12
  else
    docker image inspect carla-sim:0.9.12 >/dev/null 2>&1 || docker build -t carla-sim:0.9.12 -f "$DIR/Dockerfile.carla" "$DIR"
  fi
  echo "images ready. First 'start_docker.sh' run also builds libyuv, fetches acados and runs scons (slow)."
}

setup_ui() {
  command -v uv >/dev/null 2>&1 || { curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }
  cd "$OP_ROOT"
  git submodule update --init --recursive
  uv sync --frozen --extra tools --extra carla    # `carla` extra provides the libyuv python package
  # --minimal: skip cabana/replay/jotpluggler (they need Qt5 dev packages)
  # scons shells out to cythonize/capnpc, which live in the venv.
  # -k: openpilot's own tinygrad model targets need clang and are not used here (bukapilot runs its own modeld);
  # they may fail without blocking msgq/visionipc/cereal, which is all host_ui.py needs.
  PATH="$OP_ROOT/.venv/bin:$PATH" VIRTUAL_ENV="$OP_ROOT/.venv" .venv/bin/scons -k -j"$(nproc)" --minimal || true
  PYTHONPATH="$OP_ROOT" OPENPILOT_PREFIX=check .venv/bin/python -c "
from openpilot.cereal import messaging
from msgq.visionipc import VisionIpcServer
from openpilot.common.params import Params
from opendbc.car.structs import car" && echo "openpilot UI environment ready"
}

carla_up() { (exec 3<>"/dev/tcp/127.0.0.1/$CARLA_PORT") 2>/dev/null; }

# The port can be open while the server is hung (stale session): only an RPC answer counts as "ready".
carla_ready() {
  carla_up || return 1
  docker run --rm --net=host --entrypoint python3 "$IMAGE" -c "
import carla
c = carla.Client('127.0.0.1', $CARLA_PORT); c.set_timeout(8.0)
c.get_server_version()" >/dev/null 2>&1
}

start_server() {
  if carla_ready; then echo "CARLA already running on :$CARLA_PORT"; return 0; fi
  if carla_up || [ -n "$(docker ps -aq --filter "name=^${CARLA_NAME}$")" ]; then
    echo "CARLA on :$CARLA_PORT is not responding (stale session?): restarting it"
    docker rm -f "$CARLA_NAME" >/dev/null 2>&1 || true
    for _ in $(seq 1 10); do carla_up || break; sleep 1; done
    if carla_up; then echo "port :$CARLA_PORT is still held by a non-docker process; free it or set CARLA_PORT" >&2; return 1; fi
  fi
  if [ "${1:-}" == "--foreground" ]; then exec "$DIR/start_carla.sh"; fi
  CARLA_DETACH=1 "$DIR/start_carla.sh"
  echo -n "waiting for CARLA on :$CARLA_PORT "
  for _ in $(seq 1 120); do
    carla_ready && { echo " up"; return 0; }
    echo -n "."; sleep 2
  done
  echo; echo "CARLA did not come up; see: docker logs $CARLA_NAME" >&2
  return 1
}

# run.sh's own options are taken out of the bridge args: --yellow-path / --no-yellow-path
# (openpilot UI draws the driving path in yellow; same as YELLOW_PATH=1|0, default off)
split_opts() {
  BRIDGE_ARGS=()
  for a in "$@"; do
    case "$a" in
      --yellow-path) YELLOW_PATH=1 ;;
      --no-yellow-path) YELLOW_PATH=0 ;;
      *) BRIDGE_ARGS+=("$a") ;;
    esac
  done
  export YELLOW_PATH="${YELLOW_PATH:-0}"
}

start_bridge() {
  split_opts "$@"
  set -- ${BRIDGE_ARGS[@]+"${BRIDGE_ARGS[@]}"}
  docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "Image $IMAGE missing: bash run.sh setup" >&2; exit 1; }
  carla_ready || { echo "CARLA is not responding on :$CARLA_PORT: bash run.sh server (or: bash run.sh all)" >&2; exit 1; }
  # start_docker.sh removes a previous container and (OP_UI=1) starts the host UI itself
  exec "$DIR/start_docker.sh" "$@"
}

stop_all() {
  pkill -TERM -f 'op_ui/host_ui.py' 2>/dev/null || true
  pkill -TERM -f 'openpilot/selfdrive/ui/ui.py' 2>/dev/null || true
  docker rm -f "$BRIDGE_NAME" "$CARLA_NAME" >/dev/null 2>&1 || true
  echo "stopped"
}

case "${1:-all}" in
  setup) setup ;;
  setup-ui) setup_ui ;;
  server) shift; start_server "${@:---foreground}" ;;
  bridge) shift; start_bridge "$@" ;;
  all)
    shift || true
    start_server
    start_bridge "$@"
    ;;
  status)
    if carla_ready; then echo "CARLA is running (:$CARLA_PORT)"
    elif carla_up; then echo "CARLA port :$CARLA_PORT is open but NOT responding (stale): bash run.sh stop"
    else echo "CARLA is not running"; fi
    [ -n "$(docker ps -q --filter "name=^${BRIDGE_NAME}$")" ] && echo "bukapilot + bridge are running" || echo "bukapilot + bridge are not running"
    ;;
  stop) stop_all ;;
  *)
    echo "Usage: $0 {setup|setup-ui|server|bridge|all|status|stop} [bridge args...]" >&2
    echo "  example: $0 all --carla-town Town04_Opt --carla-spawn-point 16" >&2
    exit 2
    ;;
esac
