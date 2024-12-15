import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64, String
from scoring_interfaces.srv import SetMarkerPosition
from tf_transformations import euler_from_quaternion
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
        
        # Robot state
        self.position = None
        self.yaw = 0.0
        self.scan_data = None
        
        # Wall following states
        self.state = 0
        self.state_dict = {
            0: 'wall_following',
            1: 'room_exploration',
            2: 'corridor_following',
            3: 'corner_handling',
            4: 'room_entry'
        }
        
        # Direction modifier based on robot ID
        self.direction_modifier = -1 if self.namespace == 'tb3_0' else 1
        
        # Region definitions
        self.regions = {
            'right': 10.0,
            'fright': 10.0,
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
            'diagonal_right': 10.0,
            'diagonal_left': 10.0
        }
        
        # Create timer for movement control
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"{self.namespace} - Robot controller initialized")

    def odom_callback(self, msg):
        """Handle odometry data"""
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = msg.ranges
        try:
            self.regions = {
                'right':  min(min([x for x in msg.ranges[180:299] if not math.isinf(x)], default=10.0), 10.0),
                'fright': min(min([x for x in msg.ranges[300:339] if not math.isinf(x)], default=10.0), 10.0),
                'front':  min(min([x for x in msg.ranges[0:20] + msg.ranges[340:359] if not math.isinf(x)], default=10.0), 10.0),
                'fleft':  min(min([x for x in msg.ranges[21:60] if not math.isinf(x)], default=10.0), 10.0),
                'left':   min(min([x for x in msg.ranges[61:179] if not math.isinf(x)], default=10.0), 10.0),
                'diagonal_right': min(min([x for x in msg.ranges[225:265] if not math.isinf(x)], default=10.0), 10.0),
                'diagonal_left': min(min([x for x in msg.ranges[95:135] if not math.isinf(x)], default=10.0), 10.0),
            }
        except Exception as e:
            self.get_logger().error(f"{self.namespace} - Error in scan_callback: {str(e)}")

    def wall_following(self):
        """Basic wall following behavior"""
        msg = Twist()
        
        # Different behavior based on robot ID
        if self.namespace == 'tb3_0':
            side_distance = self.regions['right']
            target_distance = 0.5
        else:
            side_distance = self.regions['left']
            target_distance = 0.5

        # If front is clear
        if self.regions['front'] > 0.7:
            msg.linear.x = 0.2
            error = side_distance - target_distance
            msg.angular.z = 0.5 * error * self.direction_modifier
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.direction_modifier

        return msg

    def corner_escape(self):
        """Handle corner escape"""
        msg = Twist()
        msg.linear.x = -0.1
        msg.angular.z = 0.5 * self.direction_modifier
        return msg

    def control_loop(self):
        """Main control loop"""
        if not self.scan_data:
            self.get_logger().warn(f"{self.namespace} - Waiting for sensor data...")
            return

        msg = Twist()
        
        # Basic wall following for now
        if self.regions['front'] < 0.5:  # Too close to front wall
            msg = self.corner_escape()
        else:
            msg = self.wall_following()

        # Add speed limits
        msg.linear.x = max(-0.3, min(0.3, msg.linear.x))
        msg.angular.z = max(-1.0, min(1.0, msg.angular.z))

        self.cmd_vel_pub.publish(msg)
        self.get_logger().debug(f"{self.namespace} - v: {msg.linear.x:.2f}, w: {msg.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
