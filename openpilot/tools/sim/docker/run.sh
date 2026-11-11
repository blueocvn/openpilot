#!/usr/bin/env bash
# openpilot + CARLA 0.9.16 on native Ubuntu, everything in Docker (same commands as ../carla/run.sh,
# which targets WSL2 + Windows CARLA).
#
#   bash run.sh setup                 # one time: build images, then uv sync + scons --minimal in a container
#   bash run.sh all [bridge args]     # CARLA (background) + openpilot + bridge
#   bash run.sh all --carla-town Town04_Opt --carla-spawn-point 40
#   bash run.sh server | bridge [args] | shell | status | stop
#
# Environment: CARLA_HEADLESS=1 (no CARLA window), CARLA_QUALITY=Low|Epic (default Epic),
#              SCALE=1.5 (openpilot UI window size, same as bukapilot), CARLA_PORT=2100 (kept off 2000 so it can sit next to bukapilot's 0.9.12 server),
#              OPENPILOT_UI=0 (no openpilot UI), SKIP_BUILD=1 (skip scons).
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../../../.." && pwd)"            # openpilot repo root (pyproject.toml)
IMAGE="${IMAGE:-openpilot-sim:local}"
CARLA_IMAGE_WINDOWED="carla-sim:0.9.16"
CARLA_IMAGE_HEADLESS="carlasim/carla:0.9.16"
CARLA_NAME="${CARLA_NAME:-carla16_server}"
BRIDGE_NAME="${BRIDGE_NAME:-openpilot_sim_carla}"
export CARLA_PORT="${CARLA_PORT:-2100}"
STATE="$ROOT/.carla"
PYENV="$STATE/docker-env"        # container-only Python environment, never touches the host's
mkdir -p "$STATE/logs" "$STATE/docker-cache"

[ -f "$ROOT/pyproject.toml" ] || { echo "Expected openpilot at $ROOT" >&2; exit 1; }

if [ "${CARLA_HEADLESS:-0}" != "1" ] && [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set: running headless." >&2
  CARLA_HEADLESS=1
fi
CARLA_HEADLESS="${CARLA_HEADLESS:-0}"

# op_run [extra docker flags] IMAGE [cmd...]: one-off openpilot container with the repo mounted
op_run() {
  local f=(--rm -i --net=host --gpus all --shm-size 1G -u "$(id -u):$(id -g)"
    -e HOME=/home/sim -e PYTHONPATH="$ROOT"
    -e UV_CACHE_DIR="$STATE/docker-cache" -e UV_PROJECT_ENVIRONMENT="$PYENV"
    -e UV_PYTHON_INSTALL_DIR="$STATE/docker-python"
    -v "$ROOT:$ROOT" -w "$ROOT")
  [ -t 0 ] && f=(--rm -it "${f[@]:2}")
  [ -n "${DISPLAY:-}" ] && f+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
  [ -d /dev/dri ] && f+=(--device=/dev/dri:/dev/dri)
  [ -d /dev/input ] && f+=(--device=/dev/input:/dev/input)
  docker run "${f[@]}" "$@"
}

carla_up() { (exec 3<>"/dev/tcp/127.0.0.1/$CARLA_PORT") 2>/dev/null; }
# The port can be open while the server is hung: only an RPC answer counts as ready.
carla_ready() {
  carla_up || return 1
  [ -x "$PYENV/bin/python" ] || return 0   # before setup there is no client to ask; trust the port
  docker run --rm --net=host -u "$(id -u):$(id -g)" -v "$ROOT:$ROOT" --entrypoint "$PYENV/bin/python" "$IMAGE" -c "
import carla
c = carla.Client('127.0.0.1', $CARLA_PORT); c.set_timeout(8.0)
c.get_server_version()" >/dev/null 2>&1
}

build_images() {
  docker image inspect "$IMAGE" >/dev/null 2>&1 || \
    docker build -t "$IMAGE" --build-arg USER_ID="$(id -u)" --build-arg GROUP_ID="$(id -g)" -f "$DIR/Dockerfile.openpilot" "$DIR"
  if [ "$CARLA_HEADLESS" == "1" ]; then
    docker image inspect "$CARLA_IMAGE_HEADLESS" >/dev/null 2>&1 || docker pull "$CARLA_IMAGE_HEADLESS"
  else
    docker image inspect "$CARLA_IMAGE_WINDOWED" >/dev/null 2>&1 || docker build -t "$CARLA_IMAGE_WINDOWED" -f "$DIR/Dockerfile.carla" "$DIR"
  fi
}

setup() {
  build_images
  op_run --entrypoint /bin/bash "$IMAGE" -c '
    set -e
    git lfs pull || echo "git lfs pull failed; continuing"
    uv sync --frozen --extra carla --extra tools
    export PATH="$UV_PROJECT_ENVIRONMENT/bin:$PATH" VIRTUAL_ENV="$UV_PROJECT_ENVIRONMENT"
    scons --minimal -j"$(nproc)"
    scons openpilot/tools/sim/lib/libyuv_pyx.so'
  echo "setup done"
}

start_server() {
  if carla_ready; then echo "CARLA already running on :$CARLA_PORT"; return 0; fi
  if carla_up || [ -n "$(docker ps -aq --filter "name=^${CARLA_NAME}$")" ]; then
    echo "CARLA on :$CARLA_PORT is not responding: restarting it"
    docker rm -f "$CARLA_NAME" >/dev/null 2>&1 || true
    for _ in $(seq 1 10); do carla_up || break; sleep 1; done
    if carla_up; then echo "port :$CARLA_PORT is held by another process; free it or set CARLA_PORT" >&2; return 1; fi
  fi
  build_images
  local mode=(-d) render image extra=()
  if [ "${1:-}" == "--foreground" ]; then mode=(); [ -t 0 ] && mode=(-it); fi
  if [ "$CARLA_HEADLESS" == "1" ]; then
    image="$CARLA_IMAGE_HEADLESS"; render=(-RenderOffScreen)
  else
    image="$CARLA_IMAGE_WINDOWED"; render=(-windowed -ResX="${CARLA_RES_X:-1280}" -ResY="${CARLA_RES_Y:-720}")
    extra=(-e DISPLAY="$DISPLAY" -e NVIDIA_DRIVER_CAPABILITIES=all)
    xhost +local: >/dev/null 2>&1 || true
  fi
  docker run --rm ${mode[@]+"${mode[@]}"} --name "$CARLA_NAME" --gpus all --net=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw ${extra[@]+"${extra[@]}"} "$image" \
    /bin/bash ./CarlaUE4.sh -nosound "${render[@]}" -carla-rpc-port="$CARLA_PORT" -benchmark -fps=20 \
    -quality-level="${CARLA_QUALITY:-Epic}"
  [ "${1:-}" == "--foreground" ] && return 0
  echo -n "waiting for CARLA on :$CARLA_PORT "
  for _ in $(seq 1 120); do carla_ready && { echo " up"; return 0; }; echo -n "."; sleep 2; done
  echo; echo "CARLA did not come up; see: docker logs $CARLA_NAME" >&2
  return 1
}

start_bridge() {
  [ -x "$PYENV/bin/python" ] || { echo "Not set up yet: bash run.sh setup" >&2; exit 1; }
  carla_ready || { echo "CARLA is not responding on :$CARLA_PORT: bash run.sh server (or: bash run.sh all)" >&2; exit 1; }
  docker rm -f "$BRIDGE_NAME" >/dev/null 2>&1 || true
  local ui_env=(-e CI=1); [ "${OPENPILOT_UI:-1}" == "1" ] && ui_env=()
  local args=""; for a in "$@"; do args="$args $(printf '%q' "$a")"; done
  local build="scons --minimal -j$(nproc) &&"; [ -n "${SKIP_BUILD:-}" ] && build=""
  # FINGERPRINT/SCALE/BLOCK match ../carla/run.sh: the CARLA Tesla, soundd off; journald off (needs journalctl, absent in the container)
  op_run --name "$BRIDGE_NAME" --entrypoint /bin/bash \
    -e CARLA_HOST=127.0.0.1 -e CARLA_PORT -e FINGERPRINT=TESLA_MODEL_3 -e SIMULATOR=carla \
    -e SCALE="${SCALE:-1.5}" -e BLOCK="${BLOCK:-},soundd,journald" \
    -e OPENPILOT_MANAGER_LOG="$STATE/logs/openpilot.log" \
    ${ui_env[@]+"${ui_env[@]}"} "$IMAGE" -c "
      export PATH=$PYENV/bin:\$PATH VIRTUAL_ENV=$PYENV
      $build python openpilot/tools/sim/run_bridge.py --simulator carla --carla-port $CARLA_PORT --launch-openpilot$args"
}

case "${1:-all}" in
  setup) setup ;;
  server) shift; start_server "${@:---foreground}" ;;
  bridge) shift; start_bridge "$@" ;;
  all) shift || true; start_server; start_bridge "$@" ;;
  shell) op_run --entrypoint /bin/bash "$IMAGE" ;;
  status)
    if carla_ready; then echo "CARLA is running (:$CARLA_PORT)"
    elif carla_up; then echo "CARLA port :$CARLA_PORT is open but NOT responding: bash run.sh stop"
    else echo "CARLA is not running"; fi
    if [ -n "$(docker ps -q --filter "name=^${BRIDGE_NAME}$")" ]; then echo "openpilot + bridge are running"
    else echo "openpilot + bridge are not running"; fi ;;
  stop) docker rm -f "$BRIDGE_NAME" "$CARLA_NAME" >/dev/null 2>&1 || true; echo "stopped" ;;
  *) echo "Usage: $0 {setup|server|bridge|all|shell|status|stop} [bridge args...]" >&2; exit 2 ;;
esac
