from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ================================
    # 1) ROBOT STATE PUBLISHER (TF)
    # ================================
    urdf_file = os.path.join(
        get_package_share_directory('agv_description'),
        'urdf',
        'agv.urdf.xacro'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read()
        }]
    )

    # ================================
    # 2) LIDAR (RPLIDAR)
    # ================================
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'lidar.launch.py'
            )
        )
    )

    # ================================
    # 3) IMU (W901)
    # ================================
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'imu.launch.py'
            )
        )
    )

    # ================================
    # 4) Wheel Odometry (STM32)
    # ================================
    wheel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'wheel.launch.py'
            )
        )
    )

    # ================================
    # 5) (OPTIONAL) EKF FUSION
    # ================================
    ekf_params = os.path.join(
        get_package_share_directory('agv_control'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params]
    )

    # ================================
    # 6) SLAM TOOLBOX
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
            'use_sim_time': False,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'map_frame': 'map',
            'scan_topic': '/scan'
        }]
    )

    return LaunchDescription([
        robot_state_pub,
        lidar_launch,
        imu_launch,
        wheel_launch,
        ekf_node,      # nếu chưa có ekf.yaml thì comment dòng này
        slam_node
    ])
