from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'outdoor_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), 
         glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools', 'open3d', 'numpy'],
    zip_safe=True,
    maintainer='lonestarl',
    maintainer_email='lonestarl@todo.todo',
    description='Outdoor environment 3D mapping package using LiDAR SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_slam_node = outdoor_mapping.lidar_slam_node:main',
            'enhanced_lidar_slam_node = outdoor_mapping.enhanced_lidar_slam_node:main',
        ],
    },
)
