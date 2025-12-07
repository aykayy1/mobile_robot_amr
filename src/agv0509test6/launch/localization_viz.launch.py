#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'agv0509test6' 
    pkg_share = get_package_share_directory(pkg)

    # ====== ARGS ======
    # RViz config 
    default_rviz = os.path.join(pkg_share, 'config', 'slam.rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz if os.path.exists(default_rviz) else '',
        description='Path to an RViz2 config file (.rviz)'
    )
    rviz_cfg = LaunchConfiguration('rviz_config')

    # AMCL params
    default_amcl = os.path.join(pkg_share, 'config', 'amcl.yaml')
    amcl_arg = DeclareLaunchArgument(
        'amcl_params',
        default_value=default_amcl,
        description='Path to AMCL YAML parameter file'
    )
    amcl_params = LaunchConfiguration('amcl_params')

    # Map YAML (my_map.yaml)
    default_map = '/home/hcmute/maps/my_map_tesgt.yaml'
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to map YAML (e.g., new_map.yaml)'
    )
    map_yaml = LaunchConfiguration('map')





    # ====== NODES ======
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=(['-d', rviz_cfg] if (str(rviz_cfg) != '') else []),
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # Map server (Nav2)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': map_yaml}
        ]
    )

    # AMCL (Nav2)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': False}]
    )

    # Lifecycle manager 
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



    return LaunchDescription([
        #rviz_arg,
        amcl_arg,
        map_arg,
        #rviz,
        map_server,
        amcl,
        lifecycle_manager
    ])
