import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class WallFollowerController(Node):
    def __init__(self):
        super().__init__("wallFollower")

        # Determine namespace to differentiate behavior
        self.namespace = self.get_namespace().strip("/")
        self.get_logger().info(f"Initialized WallFollowerController for namespace: {self.namespace}")

        # Subscriptions and publications
        self.sub = self.create_subscription(LaserScan, "scan", self.clbk_laser, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Service to activate/deactivate the controller
        self.srv = self.create_service(SetBool, 'wall_follower_service', self.service_callback)

        # Lidar readings
        self.lidar_front = 100.0
        self.lidar_left_front = 100.0
        self.lidar_right_front = 100.0

        # Default settings
        self.mode = 'DEPLOY'  # Start in the deploy mode
        self.active = True

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)

    def service_callback(self, request, response):
        """Callback for activating/deactivating the wall follower."""
        self.active = request.data
        response.success = True
        self.get_logger().info(f"Wall follower {'activated' if self.active else 'deactivated'}.")
        return response

    def clbk_laser(self, msg):
        """Callback for processing lidar scan data."""
        self.lidar_front = msg.ranges[0]
        self.lidar_left_front = msg.ranges[45]
        self.lidar_right_front = msg.ranges[315]

    def determine_mode(self, d):
        """Determine the robot's mode based on lidar readings."""
        if self.mode == 'DEPLOY':
            # Transition to FOLLOW_WALL when a wall or obstacle is detected
            if self.lidar_front < d or self.lidar_left_front < d or self.lidar_right_front < d:
                self.get_logger().info("Wall detected, transitioning to FOLLOW_WALL mode.")
                return 'FOLLOW_WALL'
            return 'DEPLOY'

        # Regular wall-following logic
        if self.lidar_front > d and self.lidar_left_front > d and self.lidar_right_front > d:
            return 'DEFAULT'
        elif self.lidar_front < d and self.lidar_left_front > d and self.lidar_right_front > d:
            return 'FRONT'
        elif self.lidar_front > d and self.lidar_left_front > d and self.lidar_right_front < d:
            return 'FRONT_RIGHT'
        elif self.lidar_front > d and self.lidar_left_front < d and self.lidar_right_front > d:
            return 'FRONT_LEFT'
        elif self.lidar_front < d and self.lidar_left_front > d and self.lidar_right_front < d:
            return 'FRONT_AND_FRONT_RIGHT'
        elif self.lidar_front < d and self.lidar_left_front < d and self.lidar_right_front > d:
            return 'FRONT_AND_FRONT_LEFT'
        elif self.lidar_front < d and self.lidar_left_front < d and self.lidar_right_front < d:
            return 'FRONT_AND_FRONT_LEFT_AND_FRONT_RIGHT'
        elif self.lidar_front > d and self.lidar_left_front < d and self.lidar_right_front < d:
            return 'FRONT_LEFT_AND_FRONT_RIGHT'
        else:
            return 'FAULT'

    def timer_callback(self):
        """Control loop for wall following."""
        if not self.active:
            # Stop the robot if deactivated
            self.get_logger().info("Wall follower is deactivated. Robot is stopped.")
            self.publish_stop()
            return

        d = 0.8  # Threshold distance
        self.mode = self.determine_mode(d)
        self.get_logger().info(f"Current mode: {self.mode}")

        msg = Twist()

        if self.mode == 'DEPLOY':
            # Move straight to exit the starting area
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.pub.publish(msg)
            return  # Exit after deploying logic

        if self.mode == 'DEFAULT':
            # Move straight but add a slight turn to avoid drifting into open space
            msg.linear.x = 0.3
            msg.angular.z = -0.1 if "tb3_0" in self.namespace else 0.1

        elif self.mode == 'FRONT':
            # Turn away from the wall in front
            msg.linear.x = 0.0
            msg.angular.z = 0.2 if "tb3_0" in self.namespace else -0.2

        elif self.mode == 'FRONT_RIGHT':
            # Adjust slightly to the left
            msg.linear.x = 0.3
            msg.angular.z = 0.1

        elif self.mode == 'FRONT_LEFT':
            # Adjust slightly to the right
            msg.linear.x = 0.3
            msg.angular.z = 0.3

        elif self.mode in ['FRONT_AND_FRONT_RIGHT', 'FRONT_AND_FRONT_LEFT',
                           'FRONT_AND_FRONT_LEFT_AND_FRONT_RIGHT', 'FRONT_LEFT_AND_FRONT_RIGHT']:
            # Reverse slightly and turn to avoid collision
            msg.linear.x = -0.1
            msg.angular.z = 0.3 if "tb3_0" in self.namespace else -0.3

        elif self.mode == 'FAULT':
            # Stop in case of fault
            self.get_logger().warn("Fault detected! Stopping the robot.")
            self.publish_stop()
            return

        self.pub.publish(msg)

    def publish_stop(self):
        """Publish a stop message to halt the robot."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    controller = WallFollowerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
