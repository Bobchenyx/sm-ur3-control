"""Launch UR3 teleop stack: ur_robot_driver + MoveIt Servo + rosbridge + twist relay."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=os.environ.get("ROBOT_IP", "192.168.1.2"),
        description="IP address of the UR3 robot",
    )

    # UR robot driver
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("ur_robot_driver"),
            "/launch/ur_control.launch.py",
        ]),
        launch_arguments={
            "ur_type": "ur3",
            "robot_ip": robot_ip,
            "launch_rviz": "false",
            "use_fake_hardware": "false",
        }.items(),
    )

    # MoveIt Servo
    servo_params = {
        "moveit_servo": {
            "use_gazebo": False,
            "command_in_type": "speed_units",
            "scale": {
                "linear": 0.3,
                "rotational": 0.5,
                "joint": 0.5,
            },
            "publish_period": 0.008,  # 125 Hz
            "incoming_command_timeout": 0.15,
            "command_out_type": "trajectory_msgs/JointTrajectory",
            "publish_joint_positions": True,
            "publish_joint_velocities": True,
            "robot_link_command_frame": "base_link",
            "ee_frame_name": "tool0",
            "planning_frame": "base_link",
        }
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[servo_params],
        output="screen",
    )

    # rosbridge for macOS WebSocket connection
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("rosbridge_server"),
            "/launch/rosbridge_websocket_launch.xml",
        ]),
    )

    # Twist relay (safety watchdog)
    twist_relay = Node(
        package="sm_teleop",
        executable="twist_relay_node",
        parameters=[{
            "input_topic": "/spacemouse/twist",
            "output_topic": "/servo_node/delta_twist_cmds",
            "timeout_sec": 0.15,
            "frame_id": "base_link",
            "publish_rate": 125.0,
        }],
        output="screen",
    )

    return LaunchDescription([
        robot_ip_arg,
        ur_driver_launch,
        servo_node,
        rosbridge_launch,
        twist_relay,
    ])
