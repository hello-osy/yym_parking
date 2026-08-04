from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


BAG_DIR = '/Users/hello-osy/OSY_STUDY/260722_2/ai-autonomous-driving-competition-2026'


def find_bags(bag_dir):
    root = Path(bag_dir)
    if not root.is_dir():
        return []
    return sorted({p.parent for p in root.rglob('metadata.yaml')}, key=lambda p: p.name)


def choose_bag(bags):
    print(f'Available bags in {BAG_DIR}:')
    for i, bag in enumerate(bags, start=1):
        print(f'  {i}. {bag.name}')
    while True:
        choice = input(f'Select a bag [1-{len(bags)}]: ').strip()
        if choice.isdigit() and 1 <= int(choice) <= len(bags):
            return bags[int(choice) - 1]
        print('Invalid selection, try again.')


def generate_launch_description():
    share = Path(get_package_share_directory('sensor_topic'))
    bags = find_bags(BAG_DIR)

    if not bags:
        return LaunchDescription([
            LogInfo(
                msg=f'No rosbag found under BAG_DIR={BAG_DIR}. '
                    'Set BAG_DIR in sensor_utils/launch/bag_visualization.launch.py first.'
            ),
        ])

    bag_path = str(choose_bag(bags))

    return LaunchDescription([
        LogInfo(msg=f'Playing bag: {bag_path}'),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path],
            output='screen',
        ),
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
