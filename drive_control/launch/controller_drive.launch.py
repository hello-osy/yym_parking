from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sensor_topic_share = Path(get_package_share_directory('sensor_topic'))

    return LaunchDescription([
        Node(
            package='sensor_topic',
            executable='arduino_communication_node',
            output='screen',
            parameters=[{
                'port': 'auto',
                'max_steer_pwm': 150,
                'max_drive_pwm': 140,
            }],
        ),
        Node(
            package='sensor_topic',
            executable='controller_node',
            output='screen',
            parameters=[str(sensor_topic_share / 'config' / 'controller.yaml')],
        ),
        # 조이스틱(Joy) -> 공통 제어 토픽(/motor_control) 변환
        Node(
            package='sensor_utils',
            executable='joy_to_motor_node',
            output='screen',
            parameters=[{
                'steer_axis': 3,
                'drive_axis': 1,
                'invert_steer_axis': False,
                'invert_drive_axis': True,
                'deadzone': 0.2,
                'max_speed': 140,
                'max_steer': 45,
            }],
        ),
        Node(
            package='drive_control',
            executable='drive_control_node',
            output='screen',
            parameters=[{
                'max_drive_pwm': 140,
                'steer_pwm': 150,
                'steer_max_angle_deg': 45.0,
                'steer_angle_tolerance_deg': 1.0,
                'steering_feedback_timeout_sec': 0.5,
            }],
        ),
    ])
