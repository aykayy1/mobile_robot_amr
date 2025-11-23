from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():




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

            'odom_frame': 'odom',
            'base_frame': 'Base_footprint',
            'map_frame': 'map',
            'scan_topic': '/scan'
        }]
    )



    return LaunchDescription([
        # nếu chưa có ekf.yaml thì comment dòng này
        slam_node
    ])
