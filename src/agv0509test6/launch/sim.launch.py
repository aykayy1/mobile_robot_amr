#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg)

    # ===== Args =====
    default_world = '/home/anhkhoa/Mobile_robot/agv_ws/simple_corridor.world'
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to a .world/.sdf file'
    )
    world_cfg = LaunchConfiguration('world')

    # ===== URDF =====
    urdf_path = os.path.join(pkg_share, 'urdf', 'agv0509test6.urdf')
    with open(urdf_path, 'r') as f:
        urdf_text = f.read()
    urdf_resolved = urdf_text.replace('package://' + pkg, 'file://' + pkg_share)

    # ===== EKF config =====
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # ===== Gazebo Sim (ros_gz_sim) =====
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': world_cfg,          # path to .world/.sdf
            'gui': 'true',
            'server': 'true',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ===== Robot State Publisher =====
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_text, 'use_sim_time': True}],
        output='screen'
    )

    joint_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
)

    # ===== Spawn robot into Gazebo =====
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', pkg,
            '-string', urdf_resolved,
            '-x', '0.0', '-y', '0.0', '-z', '0.3',
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # ===== ROS <-> Gazebo bridge =====
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/world/simple_corridor/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        remappings=[('/world/simple_corridor/clock', '/clock')],
        output='screen'
    )

    # ===== Static TFs =====
    #left_wheel_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_tf_left_wheel',
    #    arguments=['-0.0055001', '0.27899', '0.0075', '0', '0', '0', 'Base_Link', 'Left_Wheel']
    #)
    #right_wheel_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_tf_right_wheel',
    #    arguments=['-0.0055011', '-0.30701', '0.0075', '0', '0', '0', 'Base_Link', 'Right_Wheel']
    #)

    # ===== EKF =====
    ekf = Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter',
         output='screen',
         parameters=[ekf_config, {'use_sim_time': True}]
     )

    return LaunchDescription([
        world_arg,
        gz_sim,
        rsp,
        joint_pub,
        spawn,
        bridge,
        ekf,
    #    left_wheel_tf,
    #    right_wheel_tf,
    ])