#!/bin/bash
set -e

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

echo "[Docker] ROS2 ${ROS_DISTRO} ready"
echo "[Docker] Robot IP: ${ROBOT_IP:-192.168.1.2}"

exec "$@"
