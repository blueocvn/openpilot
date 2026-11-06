#!/bin/bash
# Run bukapilot + CARLA bridge in the bukapilot-sim:local container.
# The bukapilot tree is bind-mounted at the same path and built with scons on first run.
# Extra args are passed to bridge.py (e.g. --town Town04_Opt --spawn_point 16 --low_quality).
# Set SHELL_ONLY=1 to just get a shell in the container.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
IMAGE="${IMAGE:-bukapilot-sim:local}"

xhost +local: > /dev/null

# A previous run may still be alive (e.g. the terminal was closed without pressing q),
# which blocks the container name and keeps the old GUI around. Remove it first.
if [ -n "$(docker ps -aq --filter name=^bukapilot_sim$)" ]; then
  echo "Removing previous bukapilot_sim container..."
  docker rm -f bukapilot_sim > /dev/null
fi

if [ -n "$SHELL_ONLY" ]; then
  CMD="bash"
else
  CMD="cd $ROOT && scons -j$(nproc) && cd $ROOT/tools/sim && ./tmux_script.sh $*"
fi

docker run --net=host \
  --name bukapilot_sim \
  --rm -it \
  --gpus all \
  --device=/dev/dri:/dev/dri \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --shm-size 1G \
  -u "$(id -u):$(id -g)" \
  -e HOME=/home/sim \
  -e DISPLAY="$DISPLAY" \
  -e PYTHONPATH="$ROOT" \
  -v "$ROOT:$ROOT" \
  -w "$ROOT/tools/sim" \
  "$IMAGE" \
  /bin/bash -c "$CMD"
