from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('debug_view', default_value='true'),
        DeclareLaunchArgument('start_mode', default_value='recognition'),
        # Sensor drivers are owned by sensor_topic/sensors.launch.py.
        # This launch only consumes their topics; it must not reopen camera,
        # LiDAR, or Arduino devices.
        Node(
            package='parking',
            executable='parking_node_yym',
            name='parking_node_yym',
            output='screen',
            parameters=[{
                'debug_view': ParameterValue(
                    LaunchConfiguration('debug_view'),
                    value_type=bool,
                ),
                'start_mode': LaunchConfiguration('start_mode'),
            }],
        ),
        # /motor_control target -> /arduino/motor_command PWM topic.
        Node(
            package='drive_control',
            executable='drive_control_node',
            name='parking_drive_control',
            output='screen',
            parameters=[{
                'max_drive_pwm': 130,
                'steer_pwm': 150,
                'steer_pid_kp': 4.0,
                'steer_pid_ki': 0.0,
                'steer_pid_kd': 0.0,
                'steer_max_angle_deg': 45.0,
                'steer_angle_tolerance_deg': 0.675,
                'steering_feedback_timeout_sec': 0.5,
            }],
        ),
    ])
