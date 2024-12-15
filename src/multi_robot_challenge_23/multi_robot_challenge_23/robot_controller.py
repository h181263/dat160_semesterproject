import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64
from scoring_interfaces.srv import SetMarkerPosition
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
        
        # Robot state
        self.position = None
        self.yaw = 0.0
        self.scan_data = None
        self.direction_modifier = -1 if self.namespace == 'tb3_0' else 1
        self.state = 'find_wall'  # States: 'find_wall', 'follow_wall', 'turn'
        
        # Region definitions with wider angles
        self.regions = {
            'right': 10.0,
            'fright': 10.0,
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
        }
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"{self.namespace} - Robot controller initialized")

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        try:
            if self.namespace == 'tb3_0':
                self.regions = {
                    'right':  min(min([x for x in msg.ranges[260:300] if not math.isinf(x)], default=10.0), 10.0),
                    'fright': min(min([x for x in msg.ranges[301:340] if not math.isinf(x)], default=10.0), 10.0),
                    'front':  min(min([x for x in msg.ranges[341:359] + msg.ranges[0:20] if not math.isinf(x)], default=10.0), 10.0),
                    'fleft':  min(min([x for x in msg.ranges[21:60] if not math.isinf(x)], default=10.0), 10.0),
                    'left':   min(min([x for x in msg.ranges[61:100] if not math.isinf(x)], default=10.0), 10.0),
                }
            else:
                self.regions = {
                    'left':   min(min([x for x in msg.ranges[60:100] if not math.isinf(x)], default=10.0), 10.0),
                    'fleft':  min(min([x for x in msg.ranges[20:59] if not math.isinf(x)], default=10.0), 10.0),
                    'front':  min(min([x for x in msg.ranges[341:359] + msg.ranges[0:19] if not math.isinf(x)], default=10.0), 10.0),
                    'fright': min(min([x for x in msg.ranges[300:340] if not math.isinf(x)], default=10.0), 10.0),
                    'right':  min(min([x for x in msg.ranges[260:299] if not math.isinf(x)], default=10.0), 10.0),
                }
        except Exception as e:
            self.get_logger().error(f"{self.namespace} - Error in scan_callback: {str(e)}")

    def find_wall(self):
        """Initial movement to find a wall"""
        msg = Twist()
        if self.namespace == 'tb3_0':
            # TB3_0 moves forward and slightly right
            msg.linear.x = 0.2
            msg.angular.z = -0.2
            if self.regions['right'] < 0.5:  # Found right wall
                self.state = 'follow_wall'
        else:
            # TB3_1 moves forward and slightly left
            msg.linear.x = 0.2
            msg.angular.z = 0.2
            if self.regions['left'] < 0.5:  # Found left wall
                self.state = 'follow_wall'
        return msg

    def follow_wall(self):
        """Follow wall while maintaining safe distance"""
        msg = Twist()
        
        if self.namespace == 'tb3_0':  # Right wall follower
            if self.regions['front'] > 0.5:  # Front is clear
                msg.linear.x = 0.2
                if self.regions['right'] > 0.4:  # Too far from wall
                    msg.angular.z = -0.2
                elif self.regions['right'] < 0.3:  # Too close to wall
                    msg.angular.z = 0.2
                else:
                    msg.angular.z = 0.0
            else:  # Front obstacle
                self.state = 'turn'
        else:  # Left wall follower
            if self.regions['front'] > 0.5:  # Front is clear
                msg.linear.x = 0.2
                if self.regions['left'] > 0.4:  # Too far from wall
                    msg.angular.z = 0.2
                elif self.regions['left'] < 0.3:  # Too close to wall
                    msg.angular.z = -0.2
                else:
                    msg.angular.z = 0.0
            else:  # Front obstacle
                self.state = 'turn'
        
        return msg

    def turn(self):
        """Handle turning at obstacles"""
        msg = Twist()
        msg.linear.x = 0.0
        
        if self.namespace == 'tb3_0':
            msg.angular.z = 0.4  # Turn left at obstacles
            if self.regions['front'] > 0.5 and self.regions['right'] < 0.5:
                self.state = 'follow_wall'
        else:
            msg.angular.z = -0.4  # Turn right at obstacles
            if self.regions['front'] > 0.5 and self.regions['left'] < 0.5:
                self.state = 'follow_wall'
        
        return msg

    def control_loop(self):
        if not self.scan_data:
            self.get_logger().warn(f"{self.namespace} - Waiting for sensor data...")
            return

        # State machine for movement
        if self.state == 'find_wall':
            msg = self.find_wall()
        elif self.state == 'follow_wall':
            msg = self.follow_wall()
        elif self.state == 'turn':
            msg = self.turn()
        else:
            msg = Twist()
            self.get_logger().error(f"{self.namespace} - Unknown state: {self.state}")
        
        # Add speed limits
        msg.linear.x = max(-0.2, min(0.2, msg.linear.x))
        msg.angular.z = max(-0.5, min(0.5, msg.angular.z))

        self.cmd_vel_pub.publish(msg)
        self.get_logger().debug(f"{self.namespace} - State: {self.state}, v: {msg.linear.x:.2f}, w: {msg.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
