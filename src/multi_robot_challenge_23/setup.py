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
            'controller = multi_robot_challenge_23.master_controller:main',
            '0gotopoint_controller = multi_robot_challenge_23.tb3_0.go_to_point:main',
            '0wallfollower_controller = multi_robot_challenge_23.tb3_0.wall_follower:main',
            '0bug2_controller = multi_robot_challenge_23.tb3_0.bug2:main',
            '0robot_controller = multi_robot_challenge_23.tb3_0.robot:main',
            '1gotopoint_controller = multi_robot_challenge_23.tb3_1.go_to_point:main',
            '1wallfollower_controller = multi_robot_challenge_23.tb3_1.wall_follower:main',
            '1bug2_controller = multi_robot_challenge_23.tb3_1.bug2:main',
            '1robot_controller = multi_robot_challenge_23.tb3_1.robot:main',
        ],
    },
)