import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64
from scoring_interfaces.srv import SetMarkerPosition
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import math
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = 'tb3_0'
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.namespace}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, f'/{self.namespace}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.namespace}/scan', self.scan_callback, 10)
        
        # ArUco marker detection
        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            f'/{self.namespace}/aruco_markers',
            self.aruco_callback,
            10
        )
        
        # Robot state
        self.position = None
        self.yaw = 0.0
        self.scan_data = None
        self.regions = {'right': 10.0, 'fright': 10.0, 'front': 10.0, 'fleft': 10.0, 'left': 10.0}
        
        # Marker handling
        self.current_marker = None
        self.reported_markers = set()
        self.marker_client = self.create_client(SetMarkerPosition, 'set_marker_position')
        
        # Exploration state
        self.state = 'explore'  # 'explore', 'search_marker', 'verify_marker'
        self.search_timer = 0.0
        self.search_direction = 1.0  # For rotating while searching markers
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"{self.namespace} - Robot controller initialized")

    def odom_callback(self, msg):
        """Process odometry data"""
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = msg.ranges
        try:
            self.regions = {
                'right':  min(min([x for x in msg.ranges[250:290] if not math.isinf(x)], default=10.0), 10.0),
                'fright': min(min([x for x in msg.ranges[291:330] if not math.isinf(x)], default=10.0), 10.0),
                'front':  min(min([x for x in msg.ranges[331:359] + msg.ranges[0:30] if not math.isinf(x)], default=10.0), 10.0),
                'fleft':  min(min([x for x in msg.ranges[31:70] if not math.isinf(x)], default=10.0), 10.0),
                'left':   min(min([x for x in msg.ranges[71:110] if not math.isinf(x)], default=10.0), 10.0),
            }
        except Exception as e:
            self.get_logger().error(f"{self.namespace} - Error in scan_callback: {str(e)}")

    def aruco_callback(self, msg):
        """Handle ArUco marker detection"""
        if not msg.marker_ids or msg.marker_ids[0] in self.reported_markers:
            return
            
        self.current_marker = {
            'id': msg.marker_ids[0],
            'pose': msg.poses[0]
        }
        self.state = 'verify_marker'
        self.get_logger().info(f"Found marker {self.current_marker['id']}")

    def report_marker(self):
        """Report marker to scoring system"""
        if not self.current_marker or not self.marker_client.wait_for_service(timeout_sec=1.0):
            return False
            
        request = SetMarkerPosition.Request()
        request.marker_id = self.current_marker['id']
        request.marker_position = self.current_marker['pose'].position
        
        future = self.marker_client.call_async(request)
        future.add_done_callback(self.marker_response_callback)
        return True

    def marker_response_callback(self, future):
        """Handle marker reporting response"""
        try:
            response = future.result()
            if response.accepted:
                self.reported_markers.add(self.current_marker['id'])
                self.get_logger().info(f"Successfully reported marker {self.current_marker['id']}")
            else:
                self.get_logger().warn(f"Marker {self.current_marker['id']} report rejected")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
        
        self.current_marker = None
        self.state = 'explore'

    def control_loop(self):
        """Main control loop"""
        if not self.scan_data or not self.position:
            return

        msg = Twist()
        
        if self.state == 'verify_marker':
            # Stop and verify marker
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.report_marker():
                self.state = 'explore'
        
        elif self.state == 'search_marker':
            # Rotate slowly to search for markers
            msg.linear.x = 0.0
            msg.angular.z = 0.3 * self.search_direction
            self.search_timer += 0.1
            if self.search_timer > 6.28:  # Full rotation
                self.search_timer = 0.0
                self.state = 'explore'
        
        else:  # explore state
            # Basic wall following with periodic marker search
            if self.regions['front'] > 0.5:
                msg.linear.x = 0.15
                if self.namespace == 'tb3_1':  # Right wall follower
                    if self.regions['right'] > 0.4:
                        msg.angular.z = -0.2
                    elif self.regions['right'] < 0.3:
                        msg.angular.z = 0.2
                else:  # Left wall follower
                    if self.regions['left'] > 0.4:
                        msg.angular.z = 0.2
                    elif self.regions['left'] < 0.3:
                        msg.angular.z = -0.2
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.5 if self.namespace == 'tb3_1' else -0.5
                self.state = 'search_marker'
                self.search_timer = 0.0

        # Add speed limits
        msg.linear.x = max(-0.2, min(0.2, msg.linear.x))
        msg.angular.z = max(-0.5, min(0.5, msg.angular.z))
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
