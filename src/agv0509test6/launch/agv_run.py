#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ================================
    # 1) AGV DISPLAY (URDF + EKF + RVIZ)
    # ================================
    pkg = 'agv0509test6'   # giữ nguyên như file agv_display
    pkg_share = get_package_share_directory(pkg)

    # ----- RVIZ CONFIG ARG -----
    default_rviz = os.path.join(pkg_share, 'config', 'slam.rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz if os.path.exists(default_rviz) else '',
        description='Path to an RViz2 config file (.rviz)'
    )
    rviz_cfg = LaunchConfiguration('rviz_config')

    # ----- LOAD URDF -----
    urdf_file = os.path.join(pkg_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ----- EKF CONFIG -----
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Robot State Publisher
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

    # EKF: /wheel_vel + /imu -> /odom
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

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': False}]
    )

    # ================================
    # 2) SLAM TOOLBOX
    # ================================
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

    # Thứ tự trong list: agv_display trước, rồi đến slam_node
    return LaunchDescription([
        rviz_arg,
        robot_state_pub,
        ekf_odom,
        #rviz,
        slam_node
    ])
