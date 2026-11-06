#!/bin/bash
# Run bukapilot + the sim_carla bridge inside the bukapilot-sim:local image
# (built from tools/sim/Dockerfile.bukapilot). The bukapilot tree is bind-mounted
# at the same path and built with scons on first run. Start CARLA first:
# ./start_carla.sh in another terminal.
#
#   ./start_docker.sh                        # keyboard control, auto-engage
#   ./start_docker.sh --town Town04_Opt --spawn_point 16 --low_quality
#   ./start_docker.sh --duration 120         # headless smoke test, no TTY needed
#   SHELL_ONLY=1 ./start_docker.sh           # just a shell in the container
#   SKIP_BUILD=1 ./start_docker.sh           # skip scons (tree already built)
#   OP_UI=0 ./start_docker.sh                # bukapilot's own Qt UI instead of openpilot's UI
#
# By default (OP_UI=1) the UI is openpilot's current UI, run on the host with
# openpilot's .venv (op_ui/host_ui.py) and fed by lib/ui_relay.py in the container.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
IMAGE="${IMAGE:-bukapilot-sim:local}"
NAME="${NAME:-bukapilot_sim_carla}"

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Image $IMAGE not found. Build it first:"
  echo "  cd $ROOT && docker build -t $IMAGE -f tools/sim/Dockerfile.bukapilot ."
  exit 1
fi

[ -n "$DISPLAY" ] && xhost +local: > /dev/null 2>&1

# A previous run (terminal closed without q) keeps the name and the old UI alive.
if [ -n "$(docker ps -aq --filter "name=^${NAME}$")" ]; then
  echo "Removing previous $NAME container..."
  docker rm -f "$NAME" > /dev/null
fi

export OP_UI="${OP_UI:-1}"
export OP_UI_PORT="${OP_UI_PORT:-8765}"
OP_ROOT="$(cd "$ROOT/.." && pwd)"
HOST_UI_PID=""
if [ "$OP_UI" == "1" ] && [ -z "$SHELL_ONLY" ]; then
  if [ ! -x "$OP_ROOT/.venv/bin/python" ]; then
    echo "openpilot venv not found at $OP_ROOT/.venv; use OP_UI=0 for bukapilot's Qt UI."
    exit 1
  fi
  mkdir -p "$ROOT/.carla/op_ui/logs"
  PYTHONPATH="$OP_ROOT" "$OP_ROOT/.venv/bin/python" "$DIR/op_ui/host_ui.py" \
    > "$ROOT/.carla/op_ui/logs/host_ui.log" 2>&1 &
  HOST_UI_PID=$!
  trap '[ -n "$HOST_UI_PID" ] && kill $HOST_UI_PID 2>/dev/null' EXIT
fi

ARGS=""
for a in "$@"; do ARGS="$ARGS $(printf '%q' "$a")"; done

if [ -n "$SHELL_ONLY" ]; then
  CMD="bash"
else
  BUILD="scons -j$(nproc) &&"
  [ -n "$SKIP_BUILD" ] && BUILD=""
  CMD="cd $ROOT && $BUILD cd $DIR && ./run_bridge.py --launch-openpilot$ARGS"
fi

TTY_FLAGS="-i"
[ -t 0 ] && TTY_FLAGS="-it"

DEV_FLAGS=(--device=/dev/dri:/dev/dri)
[ -d /dev/input ] && DEV_FLAGS+=(--device=/dev/input:/dev/input)  # --joystick

docker run --net=host \
  --name "$NAME" \
  --rm $TTY_FLAGS \
  --gpus all \
  "${DEV_FLAGS[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --shm-size 1G \
  -u "$(id -u):$(id -g)" \
  -e HOME=/home/sim \
  -e DISPLAY="$DISPLAY" \
  -e PYTHONPATH="$ROOT" \
  -e OP_UI -e OP_UI_PORT -e CARLA_HOST -e CARLA_PORT -e CARLA_CAMERA_X -e CARLA_CAMERA_Z -e CARLA_SPECTATOR_MODE -e BLOCK \
  -v "$ROOT:$ROOT" \
  -w "$DIR" \
  "$IMAGE" \
  /bin/bash -c "$CMD"
