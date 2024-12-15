import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='leader',
            name='leader_node'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_handler',
            namespace='tb3_0',
            name='robot_handler'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='report_marker',
            namespace='tb3_0',
            name='report_marker'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='report_marker',
            namespace='tb3_1',
            name='report_marker'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='braitenberg',
            namespace='tb3_0',
            name='braitenberg'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='braitenberg',
            namespace='tb3_1',
            name='braitenberg'),
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
    ])