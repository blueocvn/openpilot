#!/bin/bash
# Shell (or a command) in the CARLA source build container, with the GPU, the host X11 display and the sources.
#
#   ./editor.sh                                   # open the CARLA editor (Vulkan on the NVIDIA GPU); same as ./editor.sh launch
#   ./editor.sh bash                              # interactive shell in the sources folder
#   ./editor.sh bash -c 'cd $UE4_ROOT && make'    # one command
#
# launch: EDITOR_FLAGS="-graphicsadapter=0" picks the GPU for Vulkan (the host also lists an AMD iGPU and llvmpipe, and
#   Unreal 4.26 can stall on the wrong one); set EDITOR_RHI=--opengl to use OpenGL instead.
#
# Environment: CARLA_SRC=/home/ubuntu/Project/carla-src (mounted at the same path, so Unreal's absolute paths stay valid),
#   SOURCE_IMAGE=carla-source:0.9.15 (built on first use from Dockerfile.carla-source), SOURCE_NAME=carla_source.
# The container uses the host network: an editor in Play listens on port 2000, like the carla_server container,
# so stop carla_server before pressing Play.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
SRC="${CARLA_SRC:-/home/ubuntu/Project/carla-src}"
IMAGE="${SOURCE_IMAGE:-carla-source:0.9.15}"

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  docker build -t "$IMAGE" --build-arg UID="$(id -u)" --build-arg GID="$(id -g)" \
    -f "$DIR/Dockerfile.carla-source" "$DIR" || exit 1
fi
mkdir -p "$SRC"
[ -n "$DISPLAY" ] && xhost +local: > /dev/null 2>&1

if [ $# -eq 0 ] || [ "$1" = launch ]; then
  set -- bash -c "cd '$SRC/carla' && make launch-only ARGS='${EDITOR_RHI:-} --editor-flags=\"${EDITOR_FLAGS:--graphicsadapter=0}\"'"
fi

NAME="${SOURCE_NAME:-carla_source}"
if [ -n "$(docker ps -aq -f "name=^${NAME}$")" ]; then
  echo "Stopping the previous $NAME container..."
  docker stop "$NAME" > /dev/null 2>&1
  docker rm -f "$NAME" > /dev/null 2>&1
fi

TTY=()
[ -t 0 ] && TTY=(-it)
exec docker run --rm "${TTY[@]}" \
  --name "$NAME" \
  --gpus all \
  --net=host \
  --shm-size=8g \
  -e UE4_ROOT="$SRC/UnrealEngine_4.26" \
  -e DISPLAY="${DISPLAY:-}" -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SRC:$SRC" \
  -w "$SRC" \
  "$IMAGE" "${@:-bash}"
