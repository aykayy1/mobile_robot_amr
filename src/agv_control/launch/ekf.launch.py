from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ekf_config = os.path.join(
        get_package_share_directory('agv_control'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        # nếu cần remap thêm thì thêm vào đây, ví dụ:
        # remappings=[
        #     ('/odometry/filtered', '/odometry/filtered'),
        # ]
    )

    return LaunchDescription([
        ekf_node
    ])

