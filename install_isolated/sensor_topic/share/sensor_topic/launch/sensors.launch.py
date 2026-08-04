from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def find_lidar_port():
    """Find a stable LiDAR serial path without ever selecting the Arduino ACM port."""
    by_id_dir = Path('/dev/serial/by-id')
    if by_id_dir.is_dir():
        for device in sorted(by_id_dir.iterdir()):
            name = device.name.lower()
            # /dev/ttyACM0 is the ultrasonic Arduino in this vehicle.
            if 'arduino' not in name:
                return str(device)

    # Most SLLidar USB adapters enumerate as ttyUSB*.  Use this only when a
    # stable by-id symlink is unavailable.
    usb_ports = sorted(Path('/dev').glob('ttyUSB*'))
    return str(usb_ports[0]) if usb_ports else None


def make_lidar_action(context, *, sllidar_share):
    configured_port = LaunchConfiguration('lidar_serial_port').perform(context)
    port = find_lidar_port() if configured_port == 'auto' else configured_port
    if not port:
        return [LogInfo(msg=(
            '[sensors] LiDAR serial device not found; skipping sllidar_node. '
            'Reconnect LiDAR USB/power, then relaunch. '
            'Arduino /dev/ttyACM0 is intentionally not used.'
        ))]

    return [
        LogInfo(msg=f'[sensors] Starting SLLidar on {port}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(sllidar_share / 'launch' / 'sllidar_a1_launch.py')
            ),
            launch_arguments={
                'serial_port': port,
                'serial_baudrate': LaunchConfiguration('lidar_baudrate').perform(context),
                'frame_id': 'laser',
            }.items(),
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('sensor_topic'))
    sllidar_share = Path(get_package_share_directory('sllidar_ros2'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_serial_port', default_value='auto',
            description='LiDAR port, or auto to select /dev/serial/by-id then ttyUSB',
        ),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200'),
        Node(
            package='sensor_topic',
            executable='camera_node',
            output='screen',
            parameters=[str(share / 'config' / 'camera.yaml')],
        ),
        OpaqueFunction(
            function=make_lidar_action,
            kwargs={'sllidar_share': sllidar_share},
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
    ])
