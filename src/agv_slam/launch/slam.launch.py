from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Lấy đường dẫn file params
    params_file = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'slam_toolbox_params.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',   # Robot thật dùng SYNC node
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': False,        # Robot thật = FALSE
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'map_frame': 'map',
                'scan_topic': '/scan'         # Nhớ chỉnh đúng topic lidar thật
            }
        ],
        remappings=[
            ('/scan', '/scan')  # Nếu lidar là /scan, giữ nguyên
        ]
    )

    return LaunchDescription([
        slam_node
    ])
