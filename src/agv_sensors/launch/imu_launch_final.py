from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_imu_node(context, *args, **kwargs):
    imu_type = LaunchConfiguration('type').perform(context)

    # map type -> executable
    exe_map = {
        'normal':  'wit_normal_ros.py',
        'modbus':  'wit_modbus_ros.py',
        'hmodbus': 'wit_hmodbus_ros.py',
        'can':     'wit_can_ros.py',
        'hcan':    'wit_hcan_ros.py',
    }

    exe = exe_map.get(imu_type, exe_map['normal'])

    imu_node = Node(
        package='wit_ros_imu',
        executable=exe,           # ROS2 dùng executable
        name='imu',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
        }]
    )

    return [imu_node]


def generate_launch_description():
    return LaunchDescription([
        # Arg type: normal/modbus/hmodbus/can/hcan
        DeclareLaunchArgument(
            'type',
            default_value='normal',
            description='type [normal, modbus, hmodbus, can, hcan]'
        ),

        # Port & baud giống ROS1
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB1',
            description='Serial port of IMU'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='9600',
            description='Baudrate of IMU'
        ),

        # Tạo node IMU theo type
        OpaqueFunction(function=generate_imu_node),

        # Node hiển thị 3D
        Node(
            package='wit_ros_imu',
            executable='display_3D_visualization.py',
            name='display_3D_visualization_node',
            output='screen'
        ),
    ])
