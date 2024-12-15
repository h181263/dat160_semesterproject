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
        
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = 'tb3_0'
        
        self.get_logger().info(f"Initializing controller for {self.namespace}")
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.namespace}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, f'/{self.namespace}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.namespace}/scan', self.scan_callback, 10)
        
        # Aruco marker subscribers
        self.marker_pose_sub = self.create_subscription(Pose, f'/{self.namespace}/marker_map_pose', 
                                                      self.marker_pose_callback, 10)
        self.marker_id_sub = self.create_subscription(Int64, f'/{self.namespace}/marker_id', 
                                                  self.marker_id_callback, 10)
        
        # Service client for reporting markers
        self.marker_client = self.create_client(SetMarkerPosition, '/set_marker_position')
        
        # Robot state
        self.position = None
        self.yaw = 0.0
        self.scan_data = None
        self.current_marker_pose = None
        self.current_marker_id = None
        self.reported_markers = set()
        
        # Wall following states
        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn',
            2: 'follow the wall',
            3: 'corner escape',
            4: 'wall end recovery',
        }
        
        # Direction modifier based on robot ID (mirrored behavior)
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

    def marker_pose_callback(self, msg):
        """Handle marker pose detection"""
        self.current_marker_pose = msg
        if self.current_marker_id is not None and self.current_marker_id not in self.reported_markers:
            self.report_marker()

    def marker_id_callback(self, msg):
        """Handle marker ID detection"""
        self.current_marker_id = msg.data
        if self.current_marker_pose is not None and self.current_marker_id not in self.reported_markers:
            self.report_marker()

    def report_marker(self):
        """Report detected marker to scoring system"""
        if not self.marker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Marker reporting service not available')
            return

        request = SetMarkerPosition.Request()
        request.marker_id = self.current_marker_id
        request.marker_position = self.current_marker_pose.position

        future = self.marker_client.call_async(request)
        future.add_done_callback(self.report_marker_callback)

    def report_marker_callback(self, future):
        """Handle marker reporting response"""
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f'Successfully reported marker {self.current_marker_id}')
                self.reported_markers.add(self.current_marker_id)
                # If this is the big fire marker (ID 4), stop and wait for other robot
                if self.current_marker_id == 4:
                    self.get_logger().info('Big fire detected, waiting for other robot')
            else:
                self.get_logger().warn(f'Marker {self.current_marker_id} report not accepted')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        # Update regions with more detailed sectors
        try:
            self.regions = {
                'right':  min(min([x for x in msg.ranges[180:299] if not math.isinf(x)], default=10.0), 10.0),
                'fright': min(min([x for x in msg.ranges[300:339] if not math.isinf(x)], default=10.0), 10.0),
                'front':  min(min([x for x in msg.ranges[0:20] + msg.ranges[340:359] if not math.isinf(x)], default=10.0), 10.0),
                'fleft':  min(min([x for x in msg.ranges[21:60] if not math.isinf(x)], default=10.0), 10.0),
                'left':   min(min([x for x in msg.ranges[61:179] if not math.isinf(x)], default=10.0), 10.0),
                # Add diagonal checks
                'diagonal_right': min(min([x for x in msg.ranges[225:265] if not math.isinf(x)], default=10.0), 10.0),
                'diagonal_left': min(min([x for x in msg.ranges[95:135] if not math.isinf(x)], default=10.0), 10.0),
            }
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
        d = 0.7  # Normal distance threshold
        d_close = 0.4  # Close distance threshold
        
        # Check if we're in a corner or at end of wall
        in_corner = (regions['front'] < d and 
                    (regions['diagonal_right'] < d or regions['diagonal_left'] < d))
        
        at_wall_end = (
            (self.namespace == 'tb3_0' and regions['right'] > 2.0 and regions['diagonal_right'] > 2.0) or
            (self.namespace == 'tb3_1' and regions['left'] > 2.0 and regions['diagonal_left'] > 2.0)
        )

        if in_corner:
            self.get_logger().info(f"{self.namespace} - Corner detected!")
            self.change_state(3)  # New state for corner escape
        elif at_wall_end:
            self.get_logger().info(f"{self.namespace} - Wall end detected!")
            self.change_state(4)  # New state for wall end handling
        else:
            # Mirror the region checks based on robot ID
            if self.namespace == 'tb3_0':
                check_side = regions['right']
                check_side_front = regions['fright']
            else:
                check_side = regions['left']
                check_side_front = regions['fleft']

            if regions['front'] > d and check_side_front > d:
                self.change_state(0)  # find wall
            elif regions['front'] < d:
                self.change_state(1)  # turn
            elif check_side < d:
                self.change_state(2)  # follow wall
            else:
                self.change_state(0)  # find wall

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.3 * self.direction_modifier  # Mirror the rotation
        return msg

    def turn(self):
        msg = Twist()
        msg.angular.z = 0.5 * self.direction_modifier  # Mirror the rotation
        return msg

    def follow_the_wall(self):
        msg = Twist()
        
        # Adjust speed based on robot ID
        if self.namespace == 'tb3_0':
            side_distance = self.regions['right']
            target_distance = 0.5
            msg.linear.x = 0.25
            # Proportional control for wall following
            error = side_distance - target_distance
            msg.angular.z = -0.5 * error * self.direction_modifier
        else:
            side_distance = self.regions['left']
            target_distance = 0.5
            msg.linear.x = 0.25
            # Proportional control for wall following
            error = side_distance - target_distance
            msg.angular.z = 0.5 * error * self.direction_modifier
        
        return msg

    def corner_escape(self):
        """Handle corner escape"""
        msg = Twist()
        msg.linear.x = -0.2  # Back up
        msg.angular.z = 1.0 * self.direction_modifier  # Turn based on robot ID
        return msg

    def wall_end_recovery(self):
        """Handle wall end recovery"""
        msg = Twist()
        msg.linear.x = 0.15
        msg.angular.z = 0.8 * self.direction_modifier  # Make a wider turn to find next wall
        return msg

    def control_loop(self):
        if not self.scan_data:
            return
        
        if 4 in self.reported_markers:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)
            return

        msg = Twist()
        if self.state == 0:
            msg = self.find_wall()
        elif self.state == 1:
            msg = self.turn()
        elif self.state == 2:
            msg = self.follow_the_wall()
        elif self.state == 3:
            msg = self.corner_escape()
        elif self.state == 4:
            msg = self.wall_end_recovery()
        else:
            self.get_logger().error(f"{self.namespace} - Unknown state!")
            return

        # Add speed limits
        msg.linear.x = max(-0.3, min(0.3, msg.linear.x))
        msg.angular.z = max(-1.0, min(1.0, msg.angular.z))

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
