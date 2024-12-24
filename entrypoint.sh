#!/bin/bash

source /opt/ros/humble/setup.bash
source /install/setup.bash

PACKAGE_NAME="${PACKAGE_NAME:-controller}"
LAUNCH_FILE="${LAUNCH_FILE:-controller.launch.py}"

echo "Launching ROS 2 with package: $PACKAGE_NAME, launch file: $LAUNCH_FILE"
exec ros2 launch "$PACKAGE_NAME" "$LAUNCH_FILE"

exec "$@"