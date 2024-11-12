import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = 'multi_robot_challenge_23'
    
    # Declare use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch the world with robots
    world_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory(package_name), 'launch'),
                    '/rescue_robots_w3.launch.py'
                ])
            )
        ]
    )
    
    # Launch wall followers for each robot
    wall_follower_1 = Node(
        package='multi_robot_challenge_23',
        executable='wall_follower',
        namespace='tb3_0',  # Outer wall follower
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
        ]
    )
    
    wall_follower_2 = Node(
        package='multi_robot_challenge_23',
        executable='wall_follower',
        namespace='tb3_1',  # Inner wall follower
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
        ]
    )
    
    # Launch marker detectors for each robot
    marker_detector_1 = Node(
        package='multi_robot_challenge_23',
        executable='marker_detector',
        namespace='tb3_0',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('aruco_markers', 'aruco_markers'),
        ]
    )
    
    marker_detector_2 = Node(
        package='multi_robot_challenge_23',
        executable='marker_detector',
        namespace='tb3_1',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('aruco_markers', 'aruco_markers'),
        ]
    )

        # Launch explorers for each robot
    explorer_1 = Node(
        package='multi_robot_challenge_23',
        executable='explorer',
        namespace='tb3_0',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
            ('map', 'map'),
        ]
    )
    
    explorer_2 = Node(
        package='multi_robot_challenge_23',
        executable='explorer',
        namespace='tb3_1',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', 'scan'),
            ('cmd_vel', 'cmd_vel'),
            ('map', 'map'),
        ]
    )

      # Launch marker pose nodes for each robot
    marker_pose_1 = Node(
        package='multi_robot_challenge_23',
        executable='marker_pose',
        namespace='tb3_0',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('aruco_markers', 'aruco_markers'),
        ]
    )
    
    marker_pose_2 = Node(
        package='multi_robot_challenge_23',
        executable='marker_pose',
        namespace='tb3_1',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('aruco_markers', 'aruco_markers'),
        ]
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
        # marker_detector_1,
        # marker_detector_2,
        marker_pose_1,
        marker_pose_2,
        # explorer_1,
        # explorer_2,
        aruco_1,
        aruco_2
    ])