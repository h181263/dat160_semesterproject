import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import inf


class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')

        self.get_logger().info('Starting maze explorer')

        # Subscriber and publisher
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameters for wall-following and maze exploration
        self.min_front_dist = 0.5
        self.side_dist_threshold = 0.4
        self.turn_speed = 0.5
        self.forward_speed = 0.3
        self.exploration_bias = 0.2  # Encourage exploration

    def scan_callback(self, msg):
        twist = Twist()

        # Get distances in specific directions
        front = min(min(msg.ranges[0:15]), min(msg.ranges[345:359]))
        left = min(msg.ranges[80:100])
        right = min(msg.ranges[260:280])

        # Behavior-based decision-making
        if front < self.min_front_dist:
            # Wall directly ahead: turn to the clearer side
            if left < right:
                twist.angular.z = -self.turn_speed  # Turn right
            else:
                twist.angular.z = self.turn_speed  # Turn left
            twist.linear.x = 0.0
        elif left < self.side_dist_threshold:
            # Too close to the left wall: steer away
            twist.linear.x = self.forward_speed
            twist.angular.z = -self.exploration_bias
        elif right < self.side_dist_threshold:
            # Too close to the right wall: steer away
            twist.linear.x = self.forward_speed
            twist.angular.z = self.exploration_bias
        else:
            # No immediate obstacles: move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        # Publish the twist message
        self.cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    maze_explorer = MazeExplorer()
    rclpy.spin(maze_explorer)
    maze_explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
