import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from scoring_interfaces.srv import SetMarkerPosition
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from std_msgs.msg import String, Int64
import json

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Get the actual namespace from ROS
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = 'tb3_0'
        
        self.get_logger().info(f"Initializing controller for {self.namespace}")
        
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
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
        }
        
        # Region definitions
        self.regions = {
            'right': 10.0,
            'fright': 10.0,
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
        }
        
        # Create timer for movement control
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"{self.namespace} - Initialization complete")

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        # Update regions with safety checks
        try:
            self.regions = {
                'right':  min(min([x for x in msg.ranges[180:299] if not math.isinf(x)], default=10.0), 10.0),
                'fright': min(min([x for x in msg.ranges[320:339] if not math.isinf(x)], default=10.0), 10.0),
                'front':  min(min([x for x in msg.ranges[0:9] + msg.ranges[350:359] if not math.isinf(x)], default=10.0), 10.0),
                'fleft':  min(min([x for x in msg.ranges[20:39] if not math.isinf(x)], default=10.0), 10.0),
                'left':   min(min([x for x in msg.ranges[60:179] if not math.isinf(x)], default=10.0), 10.0),
            }
            self.get_logger().debug(f"{self.namespace} - Regions: {self.regions}")
            self.take_action()
        except Exception as e:
            self.get_logger().error(f"{self.namespace} - Error in scan_callback: {str(e)}")

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def change_state(self, state):
        if state is not self.state:
            self.get_logger().info(f'{self.namespace} - [{str(state)}] - {str(self.state_dict[state])}')
            self.state = state

    def take_action(self):
        regions = self.regions
        d = 0.7  # Distance threshold

        self.get_logger().info(f"{self.namespace} - Current state: {self.state_dict[self.state]}")
        self.get_logger().info(f"{self.namespace} - Distances - Front: {regions['front']:.2f}, Left: {regions['left']:.2f}, Right: {regions['right']:.2f}")

        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
        
        self.get_logger().info(f"{self.namespace} - {state_description}")

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.3
        return msg

    def control_loop(self):
        if not self.scan_data:
            self.get_logger().warn(f"{self.namespace} - No scan data available")
            return
        
        msg = Twist()
        if self.state == 0:
            msg = self.find_wall()
            self.get_logger().info(f"{self.namespace} - Finding wall")
        elif self.state == 1:
            msg = self.turn_left()
            self.get_logger().info(f"{self.namespace} - Turning left")
        elif self.state == 2:
            msg = self.follow_the_wall()
            self.get_logger().info(f"{self.namespace} - Following wall")
        else:
            self.get_logger().error(f"{self.namespace} - Unknown state!")
            return

        # Debug movement commands
        self.get_logger().info(f"{self.namespace} - Publishing velocity - Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}")
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()  # Change namespace as needed
    
    try:
        rclpy.spin(controller)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        controller.cmd_vel_pub.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
