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

    # =========================================================
    # 1) LIDAR - CHẠY NGAY LẬP TỨC
    # =========================================================
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of RPLidar'
    )

    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a2m12_launch.py'   # đổi nếu dùng model khác
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'Lidar',
            'inverted': 'false',
            'angle_compensate': 'true',
        }.items()
    )

    # =========================================================
    # 2) IMU - CHẠY SAU 5 GIÂY
    # =========================================================
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
        }]
    )

    imu_delayed = TimerAction(
        period=5.0,
        actions=[imu_node]
    )

    # =========================================================
    # 3) AGV DISPLAY (URDF + EKF + RVIZ) - CHẠY SAU 10 GIÂY
    # =========================================================
    pkg = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg)

    # --- RVIZ CONFIG ARG ---
    default_rviz = os.path.join(pkg_share, 'config', 'slam.rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz if os.path.exists(default_rviz) else '',
        description='Path to an RViz2 config file (.rviz)'
    )
    rviz_cfg = LaunchConfiguration('rviz_config')

    # --- LOAD URDF ---
    urdf_file = os.path.join(pkg_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # --- EKF CONFIG ---
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

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
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'odom')
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': False}]
    )

    agv_display_delayed = TimerAction(
        period=10.0,
        actions=[
            robot_state_pub,
            ekf_odom,
            # rviz,  # nếu muốn auto mở rviz thì uncomment
        ]
    )

    # =========================================================
    # 4) SCAN FILTER - CHỈ CHẠY SAU KHI ROBOT_STATE_PUBLISHER START
    # =========================================================
    scan_filter_params = os.path.join(pkg_share, 'config', 'scan_filter.yaml')

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='front_scan_filter',
        output='screen',
        parameters=[scan_filter_params, {'use_sim_time': False}],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )

    # Khi robot_state_publisher start xong -> đợi 1s -> mới chạy scan_filter
    scan_filter_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_pub,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[scan_filter]
                )
            ]
        )
    )

    # =========================================================
    # 5) SLAM TOOLBOX - CHẠY SAU 13 GIÂY
    # =========================================================
    slam_params = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'slam_toolbox_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {
            'odom_frame': 'odom',
            'base_frame': 'Base_footprint',
            'map_frame': 'map',
            'scan_topic': '/scan'
        }]
    )

    slam_delayed = TimerAction(
        period=13.0,
        actions=[slam_node]
    )

    # =========================================================
    # RETURN LAUNCH DESCRIPTION
    # =========================================================
    return LaunchDescription([
        # arguments
        declare_lidar_port,
        declare_imu_port,
        declare_imu_baud,
        declare_imu_frame,
        rviz_arg,

        # chạy lần lượt với delay
        lidar_launch,          # t = 0s
        imu_delayed,           # t = 5s
        agv_display_delayed,   # t = 10s (start RSP + EKF)
        scan_filter_after_rsp, # chạy sau khi RSP start (≈ t=11s)
        slam_delayed,          # t = 13s
    ])
