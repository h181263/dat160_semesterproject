import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Int64
from scoring_interfaces.srv import SetMarkerPosition

class ReportMarkerController(Node):
    def __init__(self):
        super().__init__("ReportMarkerNode")

        # Create a client for the scoring service
        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        self.wait_for_service()

        # Subscribe to marker ID and pose topics
        self.sub_id = self.create_subscription(Int64, '/marker_id', self.clbk_id, 10)
        self.sub_pose = self.create_subscription(Pose, '/marker_map_pose', self.clbk_pose, 10)
        
        # Initialize marker data
        self.marker_id = -1
        self.marker_pose = Point()

        # Control loop timer
        self.timer = self.create_timer(1.0, self.control_loop)

    def clbk_id(self, msg):
        """Callback for receiving the marker ID."""
        self.marker_id = msg.data
        self.get_logger().info(f"Received marker ID: {self.marker_id}")

    def clbk_pose(self, msg):
        """Callback for receiving the marker pose."""
        self.marker_pose = msg.position
        self.get_logger().info(f"Received marker pose: {self.marker_pose}")

    def wait_for_service(self):
        """Wait for the scoring service to become available."""
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_marker_position service...')
        self.get_logger().info('Connected to /set_marker_position service.')

    def report_marker(self):
        """Send a marker position to the scoring service."""
        if self.marker_id < 0 or self.marker_id > 4:
            self.get_logger().warn(f"Invalid marker ID: {self.marker_id}. Skipping report.")
            return

        # Prepare and send the service request
        request = SetMarkerPosition.Request()
        request.marker_id = self.marker_id
        request.marker_position = self.marker_pose

        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            if self.future.result().accepted:
                self.get_logger().info(f"Successfully reported marker {self.marker_id}.")
            else:
                self.get_logger().warn(f"Marker {self.marker_id} was not accepted by the scoring service.")
        else:
            self.get_logger().error(f"Service call failed for marker {self.marker_id}.")

    def control_loop(self):
        """Control loop to report markers if valid data is available."""
        if 0 <= self.marker_id <= 4 and self.marker_pose:
            self.report_marker()

def main(args=None):
    rclpy.init(args=args)
    controller = ReportMarkerController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
