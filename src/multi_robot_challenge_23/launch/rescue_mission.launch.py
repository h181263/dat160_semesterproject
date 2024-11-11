import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'multi_robot_challenge_23'
    
    # Launch the world with robots
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'),
            '/rescue_robots_w3.launch.py'
        ])
    )
    
    # Launch wall followers for each robot
    wall_follower_1 = Node(
        package='multi_robot_challenge_23',
        executable='wall_follower',
        namespace='tb3_0',
        remappings=[
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
        ]
    )
    
    wall_follower_2 = Node(
        package='multi_robot_challenge_23',
        executable='wall_follower',
        namespace='tb3_1',
        remappings=[
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
        ]
    )
    
    # Launch fire detector
    fire_detector = Node(
        package='multi_robot_challenge_23',
        executable='fire_detector'
    )
    
    # Launch aruco detection for each robot
    aruco_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'),
            '/aruco_recognition.launch.py'
        ]),
        launch_arguments={'namespace': 'tb3_0'}.items()
    )
    
    aruco_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch'),
            '/aruco_recognition.launch.py'
        ]),
        launch_arguments={'namespace': 'tb3_1'}.items()
    )

    return LaunchDescription([
        world_launch,
        wall_follower_1,
        wall_follower_2,
        fire_detector,
        aruco_1,
        aruco_2
    ]) 