from setuptools import setup
from glob import glob
import os

package_name = 'amr_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    # cài toàn bộ file launch/*.py vào share/agv_sensors/launch
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*.py'))),

    # nếu có thư mục config (scan_filter.yaml, v.v.)
    (os.path.join('share', package_name, 'config'),
     glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Wheel odometry from dual ELD2-RS via STM32 + IMU W901C',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node đọc IMU W901C-RS232
            'imu_node = amr_sensors.imu_node:main',

            # Node đọc wheel velocity / odom
            'wheel_vel_node = amr_sensors.wheel_vel_node:main',

            'cmd_vel_to_uart = amr_sensors.cmd_vel_to_uart:main',
            
            'wheel_vel_from_stm32 = amr_sensors.wheel_vel_from_stm32:main',

            'wheel_vel_node_nav = amr_sensors.wheel_vel_node_nav:main',
            
            'hwt901b_imu = amr_sensors.hwt901b_imu:main',
            
            'imu_hwt901b = amr_sensors.imu_hwt901b:main',
            
            'evaluation_straight10m = amr_sensors.evaluation_straight10m:main',
            # Nếu SAU NÀY em có file agv_sensors/lidar_node.py thì bật lại dòng này
            # 'lidar_node = agv_sensors.lidar_node:main',
            

        ],
    },
)

