from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('sensor_topic'))
    sllidar_share = Path(get_package_share_directory('sllidar_ros2'))
    return LaunchDescription([
        Node(
            package='sensor_topic',
            executable='camera_node',
            output='screen',
            parameters=[str(share / 'config' / 'camera.yaml')],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(sllidar_share / 'launch' / 'sllidar_a1_launch.py')
            ),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '115200',
                'frame_id': 'laser',
            }.items(),
        ),
        Node(
            package='sensor_topic',
            executable='arduino_communication_node',
            output='screen',
            parameters=[{'port': 'auto'}],
        ),
        Node(
            package='sensor_topic',
            executable='ultrasonic_node',
            output='screen',
            parameters=[str(share / 'config' / 'ultrasonic.yaml')],
        ),
        Node(
            package='sensor_topic',
            executable='controller_node',
            output='screen',
            parameters=[str(share / 'config' / 'controller.yaml')],
        ),
        # /manual_controller/joy -> /motor_control [steer angle, drive PWM]
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
                'max_speed': 130,
                'max_steer': 45,
            }],
        ),
        # /motor_control -> /arduino/motor_command [steer PWM, drive PWM]
        Node(
            package='drive_control',
            executable='drive_control_node',
            output='screen',
            parameters=[{
                'max_drive_pwm': 130,
                'steer_pwm': 150,
                'steer_max_angle_deg': 45.0,
                'steer_angle_tolerance_deg': 1.0,
                'steer_raw_left': 560,
                'steer_raw_center': 490,
                'steer_raw_right': 420,
                'steering_feedback_timeout_sec': 0.5,
            }],
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
        ),
    ])
