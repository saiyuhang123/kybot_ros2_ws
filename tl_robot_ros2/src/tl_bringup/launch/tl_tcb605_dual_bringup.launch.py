import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 双臂驱动：左右两个 tl_driver 实例，命名空间隔离，分别连接不同 IP 的控制器
    # 服务/话题自动带上前缀，如 /arm_left/tl_driver/moveJ、/arm_right/tl_driver/moveJ
    config_path = os.path.join(
        get_package_share_directory("tl_driver"), "config", "tl_tcb605_config.yaml"
    )

    arm_left = Node(
        package="tl_driver",
        executable="tl_driver",
        name="tl_driver",
        namespace="arm_left",
        parameters=[config_path, {"arm_ip": "192.168.1.13"}],
        output="screen",
    )

    arm_right = Node(
        package="tl_driver",
        executable="tl_driver",
        name="tl_driver",
        namespace="arm_right",
        parameters=[config_path, {"arm_ip": "192.168.1.14"}],
        output="screen",
    )

    return LaunchDescription([arm_left, arm_right])
