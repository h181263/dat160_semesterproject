from setuptools import setup
import os
from glob import glob

package_name = 'multi_robot_challenge_23'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hvlrobotics',
    maintainer_email='hvlrobotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_recognition = multi_robot_challenge_23.marker_pose:main',
            'robot_handler = multi_robot_challenge_23.robot_handler:main',
            'leader = multi_robot_challenge_23.leader:main',
            'wall_follower = multi_robot_challenge_23.wall_follower:main',
            'fire_detector = multi_robot_challenge_23.fire_detector:main',
            'marker_detector = multi_robot_challenge_23.marker_detector:main',
            'fire_navigator = multi_robot_challenge_23.fire_navigator:main',
        ],
    },
)