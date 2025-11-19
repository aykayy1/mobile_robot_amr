from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'agv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # để ROS2 biết đây là một package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # cài package.xml
        ('share/' + package_name, ['package.xml']),

        # cài tất cả file launch/*.py
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),

        # cài tất cả file config/*.yaml (ekf.yaml ở đây)
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anhkhoa',
    maintainer_email='trankhoavt85@gmail.com',
    description='Control + localization (EKF) for AGV',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # hiện tại agv_control chưa có node python nào,
            # nên để trống. Sau này có node điều khiển motor
            # thì mình thêm vào đây.
            # ví dụ: 'motor_node = agv_control.motor_node:main',
        ],
    },
)
