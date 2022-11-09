#!/bin/bash
set -e
source "/opt/carla-ros-bridge/catkin_ws/devel/setup.bash"
source "/code/devel/setup.bash"

sleep 10

exec "$@"