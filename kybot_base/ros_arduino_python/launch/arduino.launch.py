import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros_arduino_python'),
        'config',
        'arduino.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_arduino_python',
            executable='arduino_node',
            name='arduino',
            output='screen',
            parameters=[config],
        )
    ])
