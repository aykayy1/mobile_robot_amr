from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'slam_toolbox_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',   # Dùng chế độ sync cho robot thực
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/scan', '/scan')     # nếu lidar là /scan
        ]
    )

    return LaunchDescription([
        slam_node
    ])
