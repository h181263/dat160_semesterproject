import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_0',
            name='controller_0'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller', 
            namespace='tb3_1',
            name='controller_1'),
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
    ])