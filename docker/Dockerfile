FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install UR driver, MoveIt2, rosbridge, and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ur-robot-driver \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-moveit-config \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create ROS2 workspace
WORKDIR /ros2_ws

# Copy custom teleop package
COPY ros2_ws/src /ros2_ws/src

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9090
EXPOSE 50002

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "sm_teleop", "ur3_teleop.launch.py"]
