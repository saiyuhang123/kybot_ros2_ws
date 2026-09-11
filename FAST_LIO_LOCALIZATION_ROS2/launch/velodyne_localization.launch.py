import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio_localization')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio_localiztion.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='velodyne_test.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # fast_lio_node (예시로 fastlio_mapping 실행)
    fast_lio_node = Node(
        package='fast_lio_localization',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # global_localization 노드 추가
    global_localization_node = Node(
        package='fast_lio_localization',
        executable='global_localization.py',
        name='global_localization',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # transform_fusion 노드 추가
    # OPENBLAS_NUM_THREADS=1: 50Hz 小矩阵 BLAS 调用会导致 OpenBLAS worker
    # 线程在 sched_yield 上自旋烧掉 ~2 个核, 小矩阵单线程反而更快
    transform_fusion_node = Node(
        package='fast_lio_localization',
        executable='transform_fusion.py',
        name='transform_fusion',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        additional_env={'OPENBLAS_NUM_THREADS': '1'},
        output='screen'
    )

    # global map publisher 노드 추가
    global_map_publisher_node = Node(
        package='fast_lio_localization',
        executable='global_map_publisher.py',
        name='global_map_publisher',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # EKF: 融合轮式里程计(/odom) + MID360s IMU(/livox/imu), 独占广播 odom->base_link
    # (transform_fusion 依赖此 TF; 底盘侧 publish_tf 已关闭避免冲突)
    ekf_config_file = os.path.join(package_path, 'config', 'ekf_config.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config_file,
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # base_link -> livox_frame 静态 TF (EKF 把 IMU 数据变换到 base_link 所需)
    # 实测: 雷达在 base_link 正前方 0.40m, 横向居中, 高 0.22m
    # 旋转填 0: 驱动已把点云/IMU 补偿到水平系, 不要再填底座倾角
    livox_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=['0.40', '0.0', '0.22', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    # P0-2 修复: /cloud_registered 的 frame_id 写作 "odom" 但实际是 LIO 自己的世界系 W_L，
    # 与 TF 里 EKF 的 odom 同名异系 → 障碍位置随两个里程计分歧漂移。
    # cloud_to_base_link 不查 TF，直接用 /Odometry 把点云转到 base_link 后重发，偏差恒为 0。
    cloud_to_base_link_node = Node(
        package='fast_lio_localization',
        executable='cloud_to_base_link.py',
        name='cloud_to_base_link',
        output='screen'
    )

    # 3D 点云转 2D scan, 供 Nav2 costmap 观测源使用 (代替原 2D 雷达的 /scan_fe)
    # 输入 /cloud_registered_base (frame: base_link, 已由 cloud_to_base_link 用 LIO 位姿转好)
    # target_frame 置空 → 不再做 TF 查询，避免 odom 同名异系；min/max_height 是 base_link 系下的高度
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/cloud_registered_base'),
                    ('scan', '/scan_fe')],
        parameters=[{
            'target_frame': '',
            'transform_tolerance': 0.05,
            'min_height': -0.12,
            'max_height': 1.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.6,
            'range_max': 4.5,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use),
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(global_map_publisher_node)
    ld.add_action(ekf_node)
    ld.add_action(livox_static_tf_node)
    ld.add_action(cloud_to_base_link_node)
    ld.add_action(pointcloud_to_scan_node)
    ld.add_action(rviz_node)

    return ld
