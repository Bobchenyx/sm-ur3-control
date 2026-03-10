"""Launch UR3 teleop stack:
   ur_robot_driver + MoveIt (with Servo) + rosbridge + twist relay.

   Reuses UR official launch files instead of reinventing them.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=os.environ.get("ROBOT_IP", "192.168.1.2"),
    )
    use_fake_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
    )

    # 1. UR robot driver (ros2_control, robot_state_publisher, controllers)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ur_robot_driver"),
            "/launch/ur_control.launch.py",
        ]),
        launch_arguments={
            "ur_type": "ur3",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": "false",
            "initial_joint_controller": "forward_position_controller",
            "activate_joint_controller": "true",
        }.items(),
    )

    # 2. MoveIt + Servo (UR official launch, includes move_group + servo_node)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ur_moveit_config"),
            "/launch/ur_moveit.launch.py",
        ]),
        launch_arguments={
            "ur_type": "ur3",
            "launch_rviz": "false",
            "launch_servo": "true",
            "use_sim_time": "false",
        }.items(),
    )

    # 3. rosbridge for macOS WebSocket connection
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            get_package_share_directory("rosbridge_server"),
            "/launch/rosbridge_websocket_launch.xml",
        ]),
    )

    # 4. Twist relay (safety watchdog between rosbridge and servo)
    twist_relay = Node(
        package="sm_teleop",
        executable="twist_relay_node",
        parameters=[{
            "input_topic": "/spacemouse/twist",
            "output_topic": "/servo_node/delta_twist_cmds",
            "timeout_sec": 0.15,
            "frame_id": "tool0",
            "publish_rate": 125.0,
        }],
        output="screen",
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_arg,
        ur_control_launch,
        ur_moveit_launch,
        rosbridge_launch,
        twist_relay,
    ])
