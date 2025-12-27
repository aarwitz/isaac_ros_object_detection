#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

exec "$@"
