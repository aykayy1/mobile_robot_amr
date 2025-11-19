#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg_name)

    # ---- use_sim_time arg ----
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Robot description (URDF) ----
    urdf_path = os.path.join(pkg_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_path, 'r') as f:
        urdf_text = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_text,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # ---- RPLIDAR A2 (real sensor) ----
    channel_type     = LaunchConfiguration('channel_type')
    serial_port      = LaunchConfiguration('serial_port')
    serial_baudrate  = LaunchConfiguration('serial_baudrate')
    frame_id         = LaunchConfiguration('frame_id')
    inverted         = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')
    scan_mode        = LaunchConfiguration('scan_mode')

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type':     channel_type,
            'serial_port':      serial_port,
            'serial_baudrate':  serial_baudrate,
            'frame_id':         frame_id,
            'inverted':         inverted,
            'angle_compensate': angle_compensate,
            'scan_mode':        scan_mode,
        }]
    )

    # ---- EKF (robot_localization) ----
#    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
#    ekf_node = Node(
#        package='robot_localization',
#        executable='ekf_node',
#        name='ekf_filter_node',
#        output='screen',
#        parameters=[ekf_config, {
#            'use_sim_time': use_sim_time
#        }]
#    )

    # ---- SLAM Toolbox ----
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'Base_footprint',   # nếu TF của bạn là base_link thì đổi lại
            'map_frame': 'map',
            'scan_topic': '/scan'
        }.items()
    )

    # ---- RViz2 ----
    rviz_config = os.path.join(pkg_share, 'rviz', 'rplidar_ros.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='Specifying channel type of lidar'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Specifying usb port to connected lidar'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='256000',
            description='Specifying usb port baudrate to connected lidar'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='Lidar',
            description='Specifying frame_id of lidar'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Specifying whether or not to invert scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Sensitivity',
            description='Specifying scan mode of lidar'
        ),

        rsp_node,
        joint_state_pub_node,
        rplidar_node,
        #ekf_node,
        slam_launch,
        rviz_node,
    ])
