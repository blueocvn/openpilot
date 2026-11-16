#!/bin/bash
# Start the CARLA server in Docker (CARLA_VERSION, default 0.9.15; must match the carla client
# in the bukapilot image: bukapilot-sim:carla-<version>, or bukapilot-sim:local for 0.9.12).
#
#   ./start_carla.sh                   # CARLA window on $DISPLAY: chase view of the car
#                                      # (the bridge keeps the spectator behind the ego car)
#   CARLA_HEADLESS=1 ./start_carla.sh  # off-screen, like tools/sim/start_carla.sh
#   CARLA_QUALITY=Low ./start_carla.sh
#   CARLA_DETACH=1 ./start_carla.sh    # background (docker logs -f carla_server)
#   CARLA_NO_ASSET_PATCH=1 ./start_carla.sh   # stock CARLA assets (skip carla-assets/)
#   CARLA_VERSION=0.9.12 ./start_carla.sh      # the previous CARLA release (carlasim/carla:<version>)

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
NAME="${CARLA_NAME:-carla_server}"
QUALITY="${CARLA_QUALITY:-High}"
PORT="${CARLA_PORT:-2000}"
VERSION="${CARLA_VERSION:-0.9.15}"

if [ "${CARLA_HEADLESS:-0}" != "1" ] && [ -z "$DISPLAY" ]; then
  echo "DISPLAY is not set; starting CARLA off-screen."
  CARLA_HEADLESS=1
fi

if [ "${CARLA_HEADLESS:-0}" == "1" ]; then
  IMAGE="${CARLA_IMAGE:-carlasim/carla:$VERSION}"
  RENDER=(-RenderOffScreen)
  EXTRA=()
  docker image inspect "$IMAGE" > /dev/null 2>&1 || docker pull "$IMAGE" || exit 1
else
  IMAGE="${CARLA_IMAGE:-carla-sim:$VERSION}"
  RENDER=(-windowed -ResX="${CARLA_RES_X:-1280}" -ResY="${CARLA_RES_Y:-720}")
  EXTRA=(-e DISPLAY="$DISPLAY" -e NVIDIA_DRIVER_CAPABILITIES=all)
  xhost +local: > /dev/null 2>&1
  if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
    echo "Building $IMAGE (CARLA $VERSION + display libs)..."
    docker build -t "$IMAGE" --build-arg CARLA_VERSION="$VERSION" -f "$DIR/Dockerfile.carla" "$DIR" || exit 1
  fi
fi

if [ -n "$(docker ps -aq --filter "name=^${NAME}$")" ]; then
  echo "Removing previous $NAME container..."
  docker rm -f "$NAME" > /dev/null
fi

# Patched cooked assets (carla-assets/<version>/Content/...) are mounted read-only over the same path in the container,
# e.g. the motorcycle riders' jacket texture with the custom logo (rebuild: carla-assets/build_rider_logo.py --version).
# Cooked assets belong to one CARLA release, so each release has its own folder; a release without one runs stock.
ASSETS="$DIR/carla-assets/$VERSION"
ASSET_MOUNTS=()
if [ "${CARLA_NO_ASSET_PATCH:-0}" != "1" ] && [ -d "$ASSETS/Content" ]; then
  while IFS= read -r f; do
    ASSET_MOUNTS+=(-v "$f:/home/carla/CarlaUE4/${f#$ASSETS/}:ro")
  done < <(find "$ASSETS/Content" -type f | sort)
fi

MODE=(-it)
[ "${CARLA_DETACH:-0}" == "1" ] && MODE=(-d)
[ ! -t 0 ] && [ "${CARLA_DETACH:-0}" != "1" ] && MODE=()

exec docker run --rm "${MODE[@]}" \
  --name "$NAME" \
  --gpus all \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${ASSET_MOUNTS[@]}" \
  "${EXTRA[@]}" \
  "$IMAGE" \
  /bin/bash ./CarlaUE4.sh -nosound "${RENDER[@]}" -carla-rpc-port="$PORT" -benchmark -fps=20 -quality-level="$QUALITY"
