from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # File cấu hình joystick
    joy_params = os.path.join(
        get_package_share_directory('amr_slam'),
        'config',
        'joystick.yaml'
    )

    # Node đọc tay cầm
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            joy_params,
            {"dev": "/dev/input/js0"},
        ],
        output='screen'
    )

    # Node chuyển joystick -> /cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )

    # Node chuyển /cmd_vel -> UART STM32
    cmd_vel_to_uart_node = Node(
        package='agv_sensors',
        executable='cmd_vel_to_uart',
        name='cmd_vel_to_uart',
        output='screen'
    )

    wheel_node = Node(
            package='amr_sensors',
            executable='wheel_vel_from_stm32',
            name='wheel_vel_from_stm32',
            output='screen',
            parameters=[
                {
                    'port': '/dev/ttyACM0',
                    'baud': 38400,
                    'wheel_radius': 0.10,
                    'wheel_separation': 0.636,
                    'gear_ratio': 10.0,
                }
            ]
    )


    return LaunchDescription([
        joy_node,
        teleop_node,
        cmd_vel_to_uart_node,
        wheel_node,
    ])

