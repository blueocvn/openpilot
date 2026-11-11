#!/bin/bash
# Run bukapilot + the tools/sim CARLA bridge inside the bukapilot-sim:local image (native Ubuntu + Docker).
# The bukapilot tree is bind-mounted at the same path. Start CARLA first: ./start_carla.sh
#
#   ./start_docker.sh                                   # keyboard control, auto-engage
#   ./start_docker.sh --carla-town Town04_Opt --carla-spawn-point 16 --low_quality
#   ./start_docker.sh --duration 120                    # smoke test, no TTY needed
#   SHELL_ONLY=1 ./start_docker.sh                      # just a shell in the container
#   SKIP_BUILD=1 ./start_docker.sh                      # skip scons
#   OP_UI=1 ./start_docker.sh                           # openpilot's UI on the host (needs <openpilot>/.venv)
#
# Default OP_UI=0: bukapilot's own Qt UI opens from the container via X11.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"          # bukapilot root
SIM="$ROOT/tools/sim"                     # the bridge code lives here
IMAGE="${IMAGE:-bukapilot-sim:local}"
NAME="${NAME:-bukapilot_sim_carla}"

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Image $IMAGE not found. Build it first:"
  echo "  cd $ROOT && docker build -t $IMAGE -f tools/sim-linux-docker/Dockerfile.bukapilot ."
  exit 1
fi

[ -n "$DISPLAY" ] && xhost +local: > /dev/null 2>&1

if [ -n "$(docker ps -aq --filter "name=^${NAME}$")" ]; then
  echo "Removing previous $NAME container..."
  docker rm -f "$NAME" > /dev/null
fi

export OP_UI="${OP_UI:-0}"
export OP_UI_PORT="${OP_UI_PORT:-8765}"
export CARLA_HOST="${CARLA_HOST:-127.0.0.1}"   # --net=host, CARLA runs on this machine
OP_ROOT="$(cd "$ROOT/.." && pwd)"
HOST_UI_PID=""
if [ "$OP_UI" == "1" ] && [ -z "$SHELL_ONLY" ]; then
  if [ ! -x "$OP_ROOT/.venv/bin/python" ]; then
    echo "openpilot venv not found at $OP_ROOT/.venv; use OP_UI=0 for bukapilot's Qt UI."
    exit 1
  fi
  mkdir -p "$ROOT/.carla/op_ui/logs"
  PYTHONPATH="$OP_ROOT" "$OP_ROOT/.venv/bin/python" "$SIM/op_ui/host_ui.py" \
    > "$ROOT/.carla/op_ui/logs/host_ui.log" 2>&1 &
  HOST_UI_PID=$!
  trap '[ -n "$HOST_UI_PID" ] && kill $HOST_UI_PID 2>/dev/null' EXIT
fi

ARGS=""
for a in "$@"; do ARGS="$ARGS $(printf '%q' "$a")"; done

# libyuv.a and acados .so files are git-ignored; fetch/build them once (same as tools/sim/run.sh)
PREP='
set -e
cd '"$ROOT"'
if [ ! -f third_party/libyuv/x64/lib/libyuv.a ]; then
  commit=$(awk "/git reset --hard/{print \$4}" third_party/libyuv/build.txt)
  [ -d /tmp/libyuv/.git ] || git clone https://chromium.googlesource.com/libyuv/libyuv /tmp/libyuv
  git -C /tmp/libyuv reset --hard $commit
  (cd /tmp/libyuv && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release . && make -j$(nproc) yuv)
  mkdir -p third_party/libyuv/x64/lib && cp /tmp/libyuv/libyuv.a third_party/libyuv/x64/lib/
fi
d=third_party/acados/x86_64/lib
base=https://raw.githubusercontent.com/kommuai/bukapilot/HEAD/third_party/acados/x86_64/lib
for f in libacados.so libblasfeo.so libhpipm.so; do
  [ -s $d/$f ] || curl -fL --retry 3 -o $d/$f $base/$f
done
[ "$(stat -c %s $d/libqpOASES_e.so 2>/dev/null || echo 0)" -gt 1000 ] || cp $d/libqpOASES_e.so.3.1 $d/libqpOASES_e.so
set +e
'

if [ -n "$SHELL_ONLY" ]; then
  CMD="bash"
else
  BUILD="$PREP cd $ROOT && scons -j$(nproc) &&"
  [ -n "$SKIP_BUILD" ] && BUILD=""
  CMD="$BUILD cd $SIM && python3 ./run_bridge.py --launch-openpilot$ARGS"
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
  -e ALC_GAP_TF -e CARLA_KEYS -e CARLA_WINDOW_TITLE -e OP_UI -e OP_UI_PORT -e CARLA_HOST -e CARLA_PORT -e CARLA_CAMERA_X -e CARLA_CAMERA_Z -e CARLA_SPECTATOR_MODE -e BLOCK \
  -v "$ROOT:$ROOT" \
  -w "$SIM" \
  "$IMAGE" \
  /bin/bash -c "$CMD"
