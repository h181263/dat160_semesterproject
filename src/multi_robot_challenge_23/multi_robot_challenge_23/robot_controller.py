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
        
        # Subscribe to filtered map with QoS
        self.filtered_map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            'filtered_map', 
            self.map_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
            )
        )
        
        # Robot state
        self.position = None
        self.yaw = 0.0
        self.scan_data = None
        self.direction_modifier = -1 if self.namespace == 'tb3_0' else 1
        
        # Region definitions
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
            self.regions = {
                'right':  min(min([x for x in msg.ranges[250:290] if not math.isinf(x)], default=10.0), 10.0),
                'fright': min(min([x for x in msg.ranges[291:330] if not math.isinf(x)], default=10.0), 10.0),
                'front':  min(min([x for x in msg.ranges[331:359] + msg.ranges[0:30] if not math.isinf(x)], default=10.0), 10.0),
                'fleft':  min(min([x for x in msg.ranges[31:70] if not math.isinf(x)], default=10.0), 10.0),
                'left':   min(min([x for x in msg.ranges[71:110] if not math.isinf(x)], default=10.0), 10.0),
            }
        except Exception as e:
            self.get_logger().error(f"{self.namespace} - Error in scan_callback: {str(e)}")

    def map_callback(self, msg):
        self.filtered_map = msg
        self.get_logger().info(f"{self.namespace} - Received map update")

    def wall_following(self):
        msg = Twist()
        
        # Different behavior based on robot ID
        if self.namespace == 'tb3_0':
            side_distance = self.regions['right']
            target_distance = 0.5
        else:
            side_distance = self.regions['left']
            target_distance = 0.5

        # If front is clear
        if self.regions['front'] > 0.5:
            msg.linear.x = 0.15  # Move slower
            error = side_distance - target_distance
            msg.angular.z = 0.5 * error * self.direction_modifier
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.direction_modifier

        return msg

    def control_loop(self):
        if not self.scan_data:
            self.get_logger().warn(f"{self.namespace} - Waiting for sensor data...")
            return

        msg = Twist()
        
        # Basic wall following behavior
        msg = self.wall_following()
        
        # Add speed limits
        msg.linear.x = max(-0.2, min(0.2, msg.linear.x))
        msg.angular.z = max(-0.5, min(0.5, msg.angular.z))

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
