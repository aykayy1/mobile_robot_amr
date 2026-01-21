from setuptools import setup
from glob import glob
import os

package_name = 'amr_navigation'

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

            'wheel_vel_node_nav = amr_navigation.wheel_vel_node_nav:main',
            

            # Nếu SAU NÀY em có file agv_sensors/lidar_node.py thì bật lại dòng này
            # 'lidar_node = agv_sensors.lidar_node:main',
            

        ],
    },
)

