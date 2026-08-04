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
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
        ),
    ])
