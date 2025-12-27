#!/bin/bash
# Frozen deployment - run the container

set -e

IMAGE_NAME="cleaningrobot-vision"
CONTAINER_NAME="isaac_ros_dev-aarch64-container"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# Remove any exited containers
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container if running
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo "🔗 Attaching to running container: $CONTAINER_NAME"
    docker exec -i -t --workdir /workspaces/isaac_ros-dev $CONTAINER_NAME /bin/bash
    exit 0
fi

echo "🚀 Starting new container: $CONTAINER_NAME"
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    --runtime nvidia \
    --name "$CONTAINER_NAME" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID=0 \
    -e DISPLAY=$DISPLAY \
    -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
    -v "$REPO_ROOT/isaac_ros_assets:/workspaces/isaac_ros-dev/isaac_ros_assets" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v /dev:/dev \
    -v /usr/bin/tegrastats:/usr/bin/tegrastats \
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
    -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api \
    --pid=host \
    --workdir /workspaces/isaac_ros-dev \
    ${IMAGE_NAME}:latest \
    /bin/bash --login



