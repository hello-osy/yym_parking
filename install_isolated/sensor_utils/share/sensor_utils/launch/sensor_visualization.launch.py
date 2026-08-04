from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    """Viewer-only launch.

    Sensor drivers are owned by ``sensor_topic/sensors.launch.py``.  Starting
    them here too opens the same camera, serial, and LiDAR devices twice.
    """
    share = Path(get_package_share_directory('sensor_topic'))
    return LaunchDescription([
        # All nodes below only subscribe to already-published sensor topics.
        Node(package='sensor_utils', executable='camera_viewer_node', output='screen'),
        Node(package='sensor_utils', executable='lidar_viewer_node', output='screen'),
        Node(
            package='sensor_utils',
            executable='ultrasonic_viewer_node',
            output='screen',
            parameters=[str(share / 'config' / 'ultrasonic.yaml')],
        ),
        Node(package='sensor_utils', executable='controller_viewer_node', output='screen'),
    ])
