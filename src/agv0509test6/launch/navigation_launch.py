#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg_name)

    nav2_params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='/home/hcmute/AMR/mobile_robot_amr/src/agv0509test6/config/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'smoother_server',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,

        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_planner', executable='planner_server', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_smoother', executable='smoother_server', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', output='screen',
             parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': lifecycle_nodes}])
    ])
