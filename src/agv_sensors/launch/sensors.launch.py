from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # ===== Tham số IMU =====
    declare_imu_port = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB0',
        description='Serial port of IMU'
    )

    declare_imu_baud = DeclareLaunchArgument(
        'imu_baud',
        default_value='9600',
        description='Baudrate of IMU'
    )

    declare_imu_frame = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID of IMU'
    )

    # ===== Tham số Lidar =====
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port of RPLidar'
    )

    # ===== Include imu.launch.py =====
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'imu.launch.py'
            )
        ),
        launch_arguments={
            'imu_port': LaunchConfiguration('imu_port'),
            'imu_baud': LaunchConfiguration('imu_baud'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
        }.items()
    )

    # ===== Include wheel.launch.py =====
    declare_wheel_port = DeclareLaunchArgument(
        'wheel_port',
        default_value='/dev/ttyACM0',
        description='Serial port to STM32 (wheel encoders)'
    )

    declare_wheel_baud = DeclareLaunchArgument(
        'wheel_baudrate',
        default_value='38400',
        description='Baudrate for wheel STM32'
    )

    declare_wheel_ppr = DeclareLaunchArgument(
        'wheel_encoder_ppr_motor',
        default_value='2500',
        description='Encoder pulses per rev on motor shaft'
    )

    declare_wheel_gear = DeclareLaunchArgument(
        'wheel_gear_ratio',
        default_value='10.0',
        description='Gear ratio motor:wheel'
    )

    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius (m)'
    )

    declare_wheel_sep = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.636',
        description='Wheel separation (m)'
    )

    wheel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('agv_sensors'),
                'launch',
                'wheel.launch.py'
            )
        ),
        launch_arguments={
            'port': LaunchConfiguration('wheel_port'),
            'baudrate': LaunchConfiguration('wheel_baudrate'),
            'encoder_ppr_motor': LaunchConfiguration('wheel_encoder_ppr_motor'),
            'gear_ratio': LaunchConfiguration('wheel_gear_ratio'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
        }.items()
    )

    # imu_launch, lidar_launch giữ nguyên như anh viết trước

    return LaunchDescription([
        # IMU args...
        # LIDAR args...
        declare_wheel_port,
        declare_wheel_baud,
        declare_wheel_ppr,
        declare_wheel_gear,
        declare_wheel_radius,
        declare_wheel_sep,
        # imu_launch,
        wheel_launch,
        # lidar_launch,
    ])