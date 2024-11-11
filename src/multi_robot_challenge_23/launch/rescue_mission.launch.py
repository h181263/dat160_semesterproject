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
    
    # Launch Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ])
    )
    
    # Add Gazebo ros_state node for scoring system
    gazebo_ros_state = Node(
        package='gazebo_ros',
        executable='gazebo_ros_state',
        name='gazebo_ros_state',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch the world with robots after Gazebo is ready
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
    
    # Launch scoring system after Gazebo services are available
    scoring_system = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='scoring',
                executable='scoring',
                name='scoring_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
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
        gazebo,
        gazebo_ros_state,
        world_launch,
        scoring_system,
        wall_follower_1,
        wall_follower_2,
        marker_detector_1,
        marker_detector_2,
        aruco_1,
        aruco_2
    ])