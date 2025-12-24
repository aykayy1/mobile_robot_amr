#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =========================
    # PACKAGE PATHS
    # =========================
    pkg_robot = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg_robot)

    urdf_file = os.path.join(pkg_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    default_ekf  = os.path.join(pkg_share, 'config', 'ekf.yaml')
    default_amcl = os.path.join(pkg_share, 'config', 'amcl.yaml')
    default_map  = '/home/hcmute/AMR/mobile_robot_amr/maps/maze_31.yaml'
    scan_filter_params = os.path.join(pkg_share, 'config', 'scan_filter.yaml')

    # =========================
    # 1) LIDAR (t = 0s)
    # =========================
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of RPLidar'
    )

    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a2m12_launch.py'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'Lidar',
            'inverted': 'false',
            'angle_compensate': 'true',
            # nếu driver hỗ trợ thì bật để giảm lệch time:
            'use_system_time': 'true',
        }.items()
    )

    # =========================
    # 2) IMU (t = 5s)
    # =========================
    declare_imu_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Serial port of IMU'
    )
    declare_imu_baud = DeclareLaunchArgument(
        'imu_baud',
        default_value='115200',
        description='Baudrate of IMU'
    )
    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID of IMU'
    )

    imu_node = Node(
        package='agv_sensors',
        executable='khoa',
        name='hwt901b_modbus',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': LaunchConfiguration('imu_baud'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'use_sim_time': False
        }]
    )

    imu_delayed = TimerAction(period=5.0, actions=[imu_node])

    # =========================
    # 3) ROBOT_STATE_PUBLISHER + EKF (t = 8s)
    # =========================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom_node',
        output='screen',
        parameters=[default_ekf, {'use_sim_time': False}],
        remappings=[('odometry/filtered', 'odom')]
    )

    rsp_and_ekf_delayed = TimerAction(
        period=8.0,
        actions=[robot_state_pub, ekf_odom]
    )

    # =========================
    # 4) SCAN FILTER
    #    - chạy sau RSP start 2s
    # =========================
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='front_scan_filter',
        output='screen',
        parameters=[scan_filter_params, {'use_sim_time': False}],
        remappings=[('scan', '/scan'),
                    ('scan_filtered', '/scan_filtered')]
    )

    # =========================
    # 5) MAP_SERVER + AMCL + LIFECYCLE
    #    - chạy sau scan_filter 5s
    # =========================
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to map YAML'
    )
    amcl_arg = DeclareLaunchArgument(
        'amcl_params',
        default_value=default_amcl,
        description='Path to AMCL YAML'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': LaunchConfiguration('map')}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('amcl_params'), {'use_sim_time': False}],
        # nếu muốn AMCL dùng scan_filtered thì mở:
        remappings=[('scan', '/scan_filtered')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Event chain:
    # RSP start -> wait 2s -> start scan_filter
    # scan_filter start -> wait 5s -> start map_server + amcl + lifecycle
    localization_after_filter = RegisterEventHandler(
        OnProcessStart(
            target_action=scan_filter,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[map_server, amcl, lifecycle_manager]
                )
            ]
        )
    )

    scan_filter_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_pub,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[scan_filter]
                )
            ]
        )
    )

    # =========================
    # RETURN LaunchDescription
    # =========================
    return LaunchDescription([
        declare_lidar_port,
        declare_imu_port,
        declare_imu_baud,
        declare_imu_frame,
        map_arg,
        amcl_arg,

        lidar_launch,            # t=0
        imu_delayed,             # t=5
        rsp_and_ekf_delayed,     # t=8
        scan_filter_after_rsp,   # t≈10 (sau RSP 2s)
        localization_after_filter # t≈15 sau scan_filter
    ])
