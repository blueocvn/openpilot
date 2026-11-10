#!/bin/bash
# Start the CARLA 0.9.12 server in Docker (must match the carla==0.9.12 client
# in bukapilot-sim:local).
#
#   ./start_carla.sh                   # CARLA window on $DISPLAY: chase view of the car
#                                      # (the bridge keeps the spectator behind the ego car)
#   CARLA_HEADLESS=1 ./start_carla.sh  # off-screen, like tools/sim/start_carla.sh
#   CARLA_QUALITY=Low ./start_carla.sh
#   CARLA_DETACH=1 ./start_carla.sh    # background (docker logs -f carla_server)

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
NAME="${CARLA_NAME:-carla_server}"
QUALITY="${CARLA_QUALITY:-High}"
PORT="${CARLA_PORT:-2000}"

if [ "${CARLA_HEADLESS:-0}" != "1" ] && [ -z "$DISPLAY" ]; then
  echo "DISPLAY is not set; starting CARLA off-screen."
  CARLA_HEADLESS=1
fi

if [ "${CARLA_HEADLESS:-0}" == "1" ]; then
  IMAGE="${CARLA_IMAGE:-carlasim/carla:0.9.12}"
  RENDER=(-RenderOffScreen)
  EXTRA=()
  docker image inspect "$IMAGE" > /dev/null 2>&1 || docker pull "$IMAGE" || exit 1
else
  IMAGE="${CARLA_IMAGE:-carla-sim:0.9.12}"
  RENDER=(-windowed -ResX="${CARLA_RES_X:-1280}" -ResY="${CARLA_RES_Y:-720}")
  EXTRA=(-e DISPLAY="$DISPLAY" -e NVIDIA_DRIVER_CAPABILITIES=all)
  xhost +local: > /dev/null 2>&1
  if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
    echo "Building $IMAGE (CARLA 0.9.12 + display libs)..."
    docker build -t "$IMAGE" -f "$DIR/Dockerfile.carla" "$DIR" || exit 1
  fi
fi

if [ -n "$(docker ps -aq --filter "name=^${NAME}$")" ]; then
  echo "Removing previous $NAME container..."
  docker rm -f "$NAME" > /dev/null
fi

MODE=(-it)
[ "${CARLA_DETACH:-0}" == "1" ] && MODE=(-d)
[ ! -t 0 ] && [ "${CARLA_DETACH:-0}" != "1" ] && MODE=()

exec docker run --rm "${MODE[@]}" \
  --name "$NAME" \
  --gpus all \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${EXTRA[@]}" \
  "$IMAGE" \
  /bin/bash ./CarlaUE4.sh -nosound "${RENDER[@]}" -carla-rpc-port="$PORT" -benchmark -fps=20 -quality-level="$QUALITY"
