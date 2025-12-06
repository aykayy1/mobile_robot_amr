from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():

    pkg = 'agv0509test6'
    pkg_share = get_package_share_directory(pkg)

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of RPLidar'
    )

    # Đường dẫn tới launch của rplidar_ros
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



    return LaunchDescription([
        declare_lidar_port,
        lidar_launch,
    ])
