#!/bin/bash
set -e
source "/opt/carla-ros-bridge/catkin_ws/devel/setup.bash"

echo "Waiting for other tasks to be up"
sleep 5

exec "$@"