import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_0',
            name='controller_0',
            remappings=[
                ('cmd_vel', '/tb3_0/cmd_vel'),
                ('odom', '/tb3_0/odom'),
                ('scan', '/tb3_0/scan'),
                ('marker_map_pose', '/tb3_0/marker_map_pose'),
                ('marker_id', '/tb3_0/marker_id')
            ]),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_1',
            name='controller_1',
            remappings=[
                ('cmd_vel', '/tb3_1/cmd_vel'),
                ('odom', '/tb3_1/odom'),
                ('scan', '/tb3_1/scan'),
                ('marker_map_pose', '/tb3_1/marker_map_pose'),
                ('marker_id', '/tb3_1/marker_id')
            ]),
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
    ])