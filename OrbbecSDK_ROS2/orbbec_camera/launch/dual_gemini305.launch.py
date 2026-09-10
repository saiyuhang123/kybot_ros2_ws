import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 双 Gemini 305：左/右手各一台，按序列号区分，命名空间隔离
    # 话题形如 /camera_left/depth/image_raw、/camera_right/color/image_raw
    left_sn_arg = DeclareLaunchArgument('left_serial', default_value='CV2L360000A4')
    right_sn_arg = DeclareLaunchArgument('right_serial', default_value='CV2L360000R4')

    launch_file = os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch', 'gemini_301_series.launch.py'
    )

    camera_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'camera_name': 'camera_left',
            'serial_number': LaunchConfiguration('left_serial'),
        }.items(),
    )

    camera_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'camera_name': 'camera_right',
            'serial_number': LaunchConfiguration('right_serial'),
        }.items(),
    )

    return LaunchDescription([left_sn_arg, right_sn_arg, camera_left, camera_right])
