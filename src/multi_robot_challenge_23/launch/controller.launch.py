from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodes for tb3_0
    bug2_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='0bug2_controller',
        #namespace='tb3_0',
        name='bug2',
        output='screen'
    )
    go_to_point_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='0gotopoint_controller',
        #namespace='tb3_0',
        name='go_to_point',
        output='screen'
    )
    robot_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='0robot_controller',
        #namespace='tb3_0',
        name='robot',
        output='screen'
    )
    wall_follower_tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='0wallfollower_controller',
        #namespace='tb3_0',
        name='wall_follower',
        output='screen'
    )

    # Nodes for tb3_1
    bug2_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='1bug2_controller',
        #namespace='tb3_1',
        name='bug2',
        output='screen'
    )
    go_to_point_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='1gotopoint_controller',
        #namespace='tb3_1',
        name='go_to_point',
        output='screen'
    )
    robot_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='1robot_controller',
        #namespace='tb3_1',
        name='robot',
        output='screen'
    )
    wall_follower_tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='1wallfollower_controller',
        #namespace='tb3_1',
        name='wall_follower',
        output='screen'
    )

    # Robot handler

    robot_handler = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        name='robothandler',
        output='screen'
    )

    return LaunchDescription([
        bug2_tb3_0,
        go_to_point_tb3_0,
        robot_tb3_0,
        wall_follower_tb3_0,
        bug2_tb3_1,
        go_to_point_tb3_1,
        robot_tb3_1,
        wall_follower_tb3_1
    ])