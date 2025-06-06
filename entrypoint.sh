#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

PACKAGE_NAME="${PACKAGE_NAME:-swarm_robot_launch}"
LAUNCH_FILE="${LAUNCH_FILE:-swarm_robot_a3.launch.py}"

echo "Launching ROS 2 package: $PACKAGE_NAME, launch file: $LAUNCH_FILE"
exec ros2 launch "$PACKAGE_NAME" "$LAUNCH_FILE"

exec "$@"