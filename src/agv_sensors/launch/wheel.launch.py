from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port to STM32 (wheel encoders)'
    )

    declare_baud = DeclareLaunchArgument(
        'baudrate',
        default_value='38400',
        description='Baudrate for STM32 serial'
    )

    declare_ppr = DeclareLaunchArgument(
        'encoder_ppr_motor',
        default_value='2500',
        description='Encoder pulses per revolution on motor shaft'
    )

    declare_gear = DeclareLaunchArgument(
        'gear_ratio',
        default_value='10.0',
        description='Gear ratio motor : wheel'
    )

    declare_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius (m)'
    )

    declare_sep = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.636',
        description='Distance between left and right wheel centers (m)'
    )

    wheel_node = Node(
        package='agv_sensors',
        executable='wheel_vel_node',
        name='wheel_vel_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'encoder_ppr_motor': LaunchConfiguration('encoder_ppr_motor'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_ppr,
        declare_gear,
        declare_radius,
        declare_sep,
        wheel_node
    ])
