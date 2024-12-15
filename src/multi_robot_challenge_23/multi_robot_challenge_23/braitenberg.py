import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition


class BraitenbergController(Node):
    def __init__(self):
        super().__init__('braitenberg')
        
        # Get namespace
        self.namespace = self.get_namespace()
        self.get_logger().info(f"Namespace: {self.namespace}")

        # Subscriptions and publications
        self.sub_laser = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)
        self.sub_markers = self.create_subscription(ArucoMarkers, f'/{self.namespace}/aruco_markers', self.clbk_markers, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Scoring service client
        self.score_client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.score_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for scoring service...")

        # Marker tracking
        self.current_marker = None
        self.current_marker_position = None
        self.reported_markers = set()

        # Lidar readings
        self.lidar_left_front = float('inf')
        self.lidar_right_front = float('inf')

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def clbk_laser(self, msg):
        """Update lidar readings."""
        if 0 <= 12 < len(msg.ranges) and 0 <= 348 < len(msg.ranges):
            self.lidar_left_front = msg.ranges[12] if msg.ranges[12] < msg.range_max else float('inf')
            self.lidar_right_front = msg.ranges[348] if msg.ranges[348] < msg.range_max else float('inf')

    def clbk_markers(self, msg):
        """Handle detected ArUco markers."""
        if not msg.marker_ids:
            return

        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            if marker_id not in self.reported_markers:
                self.current_marker = marker_id
                self.current_marker_position = pose.position
                self.get_logger().info(f"Detected marker {marker_id} at position {pose.position}.")
                break

    def report_marker(self):
        """Send a marker position to the scoring system."""
        if self.current_marker is None or self.current_marker_position is None:
            return

        request = SetMarkerPosition.Request()
        request.marker_id = self.current_marker
        request.marker_position = self.current_marker_position

        future = self.score_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().accepted:
            self.get_logger().info(f"Successfully reported marker {self.current_marker}.")
            self.reported_markers.add(self.current_marker)
        else:
            self.get_logger().warn(f"Failed to report marker {self.current_marker}.")

        # Reset marker tracking
        self.current_marker = None
        self.current_marker_position = None

    def timer_callback(self):
        """Control loop for navigation and marker handling."""
        vel_msg = Twist()

        # If a marker is detected, navigate towards it
        if self.current_marker_position:
            self.navigate_to_marker(vel_msg)
            return

        # Basic obstacle avoidance
        vel_msg.linear.x = 0.4
        if self.lidar_left_front < 2 and self.lidar_right_front < 2:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 if "tb3_0" in self.namespace else -0.5
            self.get_logger().info("Obstacle detected, turning.")
        elif self.lidar_left_front / max(self.lidar_right_front, 1e-5) < 1 and self.lidar_left_front < 2:
            vel_msg.angular.z = -0.5
            self.get_logger().info("Obstacle on left, turning right.")
        elif self.lidar_left_front / max(self.lidar_right_front, 1e-5) > 1 and self.lidar_right_front < 2:
            vel_msg.angular.z = 0.5
            self.get_logger().info("Obstacle on right, turning left.")

        self.pub.publish(vel_msg)

    def navigate_to_marker(self, vel_msg):
        """Navigate towards the current marker."""
        marker_x = self.current_marker_position.x
        marker_y = self.current_marker_position.y

        distance = (marker_x**2 + marker_y**2)**0.5
        self.get_logger().info(f"Navigating to marker {self.current_marker}, distance: {distance:.2f}")

        if distance > 0.5:
            # Navigate towards the marker
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0.0
        else:
            # Stop and report when close enough
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.report_marker()

        self.pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = BraitenbergController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
