#!/bin/bash
set -e

source "/opt/ros/noetic/setup.bash"
source "/catkin_ws/devel/setup.bash"

exec "$@"
