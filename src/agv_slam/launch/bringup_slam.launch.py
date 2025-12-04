#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================
    # PACKAGE PATHS
    # ============================
    pkg_sensors = get_package_share_directory('agv_sensors')
    pkg_slam    = get_package_share_directory('agv_slam')
    pkg_robot   = get_package_share_directory('agv0509test6')

    # ============================
    # LIDAR
    # ============================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )

    # ============================
    # IMU
    # ============================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensors, 'launch', 'imu.launch.py')
        ),
        launch_arguments={
            'imu_port': LaunchConfiguration('imu_port'),
            'imu_baud': LaunchConfiguration('imu_baud'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id')
        }.items()
    )

    # ============================
    # ENCODER STM32
    # ============================
    wheel_vel_node = Node(
        package='agv_sensors',
        executable='wheel_vel_from_stm32',
        name='wheel_vel_from_stm32',
        output='screen'
    )

    # ============================
    # CMD_VEL → UART STM32
    # ============================
    cmd_vel_uart_node = Node(
        package='agv_sensors',
        executable='cmd_vel_to_uart',
        name='cmd_vel_to_uart',
        output='screen'
    )

    # ============================
    # ROBOT STATE + EKF
    # ============================
    agv_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'agv_display.py')
        )
    )

    # ============================
    # SLAM TOOLBOX
    # ============================
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')
        )
    )

    # ============================
    # DECLARE LAUNCH ARGUMENTS
    # ============================
    declare_lidar = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0'
    )
    declare_imu_port = DeclareLaunchArgument(
        'imu_port', default_value='/dev/ttyUSB1'
    )
    declare_imu_baud = DeclareLaunchArgument(
        'imu_baud', default_value='9600'
    )
    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame_id', default_value='imu_link'
    )

    return LaunchDescription([
        declare_lidar,
        declare_imu_port,
        declare_imu_baud,
        declare_imu_frame,

        lidar_launch,
        imu_launch,
        wheel_vel_node,
        cmd_vel_uart_node,
        agv_display,
        slam_toolbox
    ])
