from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sensor_topic_share = Path(get_package_share_directory('sensor_topic'))
    use_hardware = LaunchConfiguration('use_hardware')
    debug_view = LaunchConfiguration('debug_view')
    debug_window_name = LaunchConfiguration('debug_window_name')
    driving_mode = LaunchConfiguration('driving_mode')
    reference_x_1lane = LaunchConfiguration('reference_x_1lane')
    reference_x_2lane = LaunchConfiguration('reference_x_2lane')
    close_confirm_samples = LaunchConfiguration('close_confirm_samples')
    clear_confirm_samples = LaunchConfiguration('clear_confirm_samples')

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='true'),
        DeclareLaunchArgument('debug_view', default_value='true'),
        DeclareLaunchArgument(
            'debug_window_name', default_value='mission_lane_main_debug'
        ),
        DeclareLaunchArgument('driving_mode', default_value='1lane'),
        DeclareLaunchArgument('reference_x_1lane', default_value='500'),
        DeclareLaunchArgument('reference_x_2lane', default_value='208'),
        DeclareLaunchArgument('close_confirm_samples', default_value='5'),
        DeclareLaunchArgument('clear_confirm_samples', default_value='3'),
        Node(
            package='sensor_topic',
            executable='arduino_communication_node',
            output='screen',
            condition=IfCondition(use_hardware),
            parameters=[{
                'port': 'auto',
                'max_steer_pwm': 150,
                'max_drive_pwm': 130,
            }],
        ),
        Node(
            package='sensor_topic',
            executable='camera_node',
            output='screen',
            condition=IfCondition(use_hardware),
            parameters=[str(sensor_topic_share / 'config' / 'camera.yaml')],
        ),
        Node(
            package='sensor_topic',
            executable='ultrasonic_node',
            output='screen',
            condition=IfCondition(use_hardware),
            parameters=[str(sensor_topic_share / 'config' / 'ultrasonic.yaml')],
        ),
        Node(
            package='lane_offset',
            executable='mission_lane_offset_node',
            output='screen',
            parameters=[{
                'debug_view': ParameterValue(debug_view, value_type=bool),
                'driving_mode': driving_mode,
                'dashed_reference_x_px_1lane': ParameterValue(
                    reference_x_1lane, value_type=int
                ),
                'dashed_reference_x_px_2lane': ParameterValue(
                    reference_x_2lane, value_type=int
                ),
            }],
        ),
        Node(
            package='lane_main',
            executable='mission_lane_main_node',
            output='screen',
            parameters=[{
                'driving_mode': driving_mode,
                'debug_view': ParameterValue(debug_view, value_type=bool),
                'debug_window_name': debug_window_name,
                'lane_change_close_confirm_samples': ParameterValue(
                    close_confirm_samples, value_type=int
                ),
                'lane_change_clear_confirm_samples': ParameterValue(
                    clear_confirm_samples, value_type=int
                ),
            }],
        ),
        Node(
            package='drive_control',
            executable='drive_control_node',
            output='screen',
            condition=IfCondition(use_hardware),
            parameters=[{
                'max_drive_pwm': 130,
                'steer_pwm': 150,
                'steering_control_mode': 'pid',
                'steer_max_angle_deg': 45.0,
                'steer_angle_tolerance_deg': 1.0,
                'steering_feedback_timeout_sec': 0.5,
            }],
        ),
    ])
