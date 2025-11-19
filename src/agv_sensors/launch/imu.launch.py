from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Serial port of IMU'
    )

    declare_baud = DeclareLaunchArgument(
        'imu_baud',
        default_value='9600',
        description='Baudrate of IMU'
    )

    declare_frame = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID of IMU'
    )

    imu_node = Node(
        package='agv_sensors',
        executable='imu_node',          # từ setup.py: imu_node = agv_sensors.imu_node:main
        name='w901_imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baud': LaunchConfiguration('imu_baud'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_baud,
        declare_frame,
        imu_node
    ])
