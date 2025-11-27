from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    joy_params = os.path.join(
        get_package_share_directory('agv_slam'),
        'config',
        'joystick.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            joy_params,
            {"dev": "/dev/input/js0"}
        ],
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel')   # xuất đúng topic
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
