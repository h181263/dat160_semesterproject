import random

class BraitenbergController(Node):
    def __init__(self):
        super().__init__('braitenberg')
        
        # Get namespace
        self.namespace = self.get_namespace().strip("/")
        self.get_logger().info(f"Namespace: {self.namespace}")

        # Subscriptions and publications
        self.sub_laser = self.create_subscription(LaserScan, f'/{self.namespace}/scan', self.clbk_laser, 10)
        self.sub_markers = self.create_subscription(ArucoMarkers, f'/{self.namespace}/aruco_markers', self.clbk_markers, 10)
        self.pub = self.create_publisher(Twist, f'/{self.namespace}/cmd_vel', 10)

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
        self.lidar_left_front = msg.ranges[90] if msg.ranges[90] < msg.range_max else float('inf')
        self.lidar_right_front = msg.ranges[270] if msg.ranges[270] < msg.range_max else float('inf')

        # Debug lidar values
        self.get_logger().info(f"Lidar - Left: {self.lidar_left_front:.2f}, Right: {self.lidar_right_front:.2f}")

    def clbk_markers(self, msg):
        """Handle detected ArUco markers."""
        if not msg.marker_ids:
            self.get_logger().info("No markers detected.")
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

        self.get_logger().info(f"Reporting marker {self.current_marker} at position {self.current_marker_position}.")
        request = SetMarkerPosition.Request()
        request.marker_id = self.current_marker
        request.marker_position = self.current_marker_position

        future = self.score_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().accepted:
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

        # Basic obstacle avoidance and exploration
        vel_msg.linear.x = 0.3
        if self.lidar_left_front < 1.0 and self.lidar_right_front < 1.0:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 if "tb3_0" in self.namespace else -0.5
            self.get_logger().info("Obstacle detected. Turning.")
        elif self.lidar_left_front < 1.0:
            vel_msg.angular.z = -0.3
            self.get_logger().info("Obstacle on left. Turning right.")
        elif self.lidar_right_front < 1.0:
            vel_msg.angular.z = 0.3
            self.get_logger().info("Obstacle on right. Turning left.")
        else:
            # Explore the environment if no obstacles
            vel_msg.angular.z = random.uniform(-0.2, 0.2)  # Random small adjustments
            self.get_logger().info("Exploring environment.")

        self.pub.publish(vel_msg)

    def navigate_to_marker(self, vel_msg):
        """Navigate towards the current marker."""
        marker_x = self.current_marker_position.x
        marker_y = self.current_marker_position.y

        distance = (marker_x**2 + marker_y**2)**0.5
        angle_to_marker = atan2(marker_y, marker_x)

        self.get_logger().info(f"Navigating to marker {self.current_marker}, distance: {distance:.2f}, angle: {angle_to_marker:.2f}")

        if distance > 0.5:
            # Adjust direction towards the marker
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = angle_to_marker * 0.5
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
