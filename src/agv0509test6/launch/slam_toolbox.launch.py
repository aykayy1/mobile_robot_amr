#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg)

    # Duong dan den file slam
    slam_params_file = '/home/anhkhoa/Mobile_robot/agv_ws/src/agv0509test6/config/slam_toolbox_params.yaml'


    
    # Goi launch cua slam_toolbox
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
            'use_sim_time': 'true',
            'odom_frame': 'odom',
            'base_frame': 'Base_footprint',
            'map_frame': 'map',
            'scan_topic': '/scan'
        }.items()
    )
    
    return LaunchDescription([scan_filter,
                                slam_launch])
