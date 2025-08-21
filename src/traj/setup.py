from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'traj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'traj_bridge.yaml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tommaso.faraci3@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processes = traj.processes:main',
            'arm_takeoff = traj.arm_takeoff:main',
            'offboard_control = traj.offboard_control:main',
            'drone_path_follower = traj.drone_path_follower:main',
            'drone_hardware_path_follower = traj.drone_hardware_path_follower:main',
            'offboard_takeoff = traj.offboard_takeoff:main',
            'visualizer = traj.visualizer:main',
            'px4_tf = traj.px4_transform_pub:main',
            'land_disarm = traj.land_disarm:main',
        ],
    },
)
