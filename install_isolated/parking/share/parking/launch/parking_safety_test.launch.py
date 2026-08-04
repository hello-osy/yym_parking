"""Safety-FSM test launch; it deliberately does not open sensor or serial devices."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    motor_topic = LaunchConfiguration("motor_topic")
    config_file = os.path.join(
        get_package_share_directory("parking"), "config", "parking.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "motor_topic", default_value="/parking/motor_control_test"
        ),
        Node(
            package="parking", executable="parking_node", name="parking_node",
            output="screen", parameters=[
                config_file,
                {
                    "motor_topic": motor_topic,
                    "require_camera_for_motion": False,
                },
            ],
        ),
        Node(
            package="drive_control", executable="drive_control_node",
            name="parking_drive_control", output="screen", parameters=[{
                "motor_control_topic": motor_topic,
                "max_drive_pwm": 130,
                "steer_pwm": 150,
                "steer_max_angle_deg": 45.0,
            }],
        ),
    ])
