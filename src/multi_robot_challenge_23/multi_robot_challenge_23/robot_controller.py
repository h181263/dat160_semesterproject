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
        
        # Aruco marker detection
        self.marker_sub = self.create_subscription(
            ArucoMarkers, 
            f'/{self.namespace}/aruco_markers', 
            self.marker_callback, 
            10
        )
        
        # Map subscription with QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'filtered_map',
            self.map_callback,
            qos_profile
        )
        
        # Service client for marker reporting
        self.marker_client = self.create_client(SetMarkerPosition, 'set_marker_position')
        
        # Robot state
        self.position = None
        self.yaw = 0.0
        self.scan_data = None
        self.filtered_map = None
        self.current_marker = None
        self.reported_markers = set()
        
        # Exploration state
        self.state = 'explore'  # States: 'explore', 'verify_marker', 'report_marker'
        self.unexplored_frontiers = []
        self.current_frontier = None
        self.direction_modifier = -1 if self.namespace == 'tb3_0' else 1
        
        # Initialize regions
        self.regions = {
            'right': 10.0,
            'fright': 10.0,
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
        }
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"{self.namespace} - Robot controller initialized")

    def marker_callback(self, msg):
        """Handle detected ArUco markers"""
        if not msg.marker_ids:
            return
            
        marker_id = msg.marker_ids[0]
        if marker_id in self.reported_markers:
            return
            
        self.current_marker = {
            'id': marker_id,
            'pose': msg.poses[0]
        }
        self.state = 'verify_marker'

    def map_callback(self, msg):
        """Process filtered map and update frontiers"""
        self.filtered_map = msg
        if not self.unexplored_frontiers:
            self.find_frontiers()

    def find_frontiers(self):
        """Find unexplored areas in the filtered map"""
        if not self.filtered_map:
            return
            
        frontiers = []
        width = self.filtered_map.info.width
        height = self.filtered_map.info.height
        
        # Split map for two robots
        start_x = 0 if self.namespace == 'tb3_0' else width//2
        end_x = width//2 if self.namespace == 'tb3_0' else width
        
        for y in range(height):
            for x in range(start_x, end_x):
                idx = x + y * width
                if self.filtered_map.data[idx] == 0:  # Free space
                    # Check if next to unexplored
                    for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            nidx = nx + ny * width
                            if self.filtered_map.data[nidx] == -1:  # Unexplored
                                world_x = x * self.filtered_map.info.resolution + self.filtered_map.info.origin.position.x
                                world_y = y * self.filtered_map.info.resolution + self.filtered_map.info.origin.position.y
                                frontiers.append((world_x, world_y))
                                break
        
        self.unexplored_frontiers = frontiers

    def report_marker(self):
        """Report marker position to scoring system"""
        if not self.current_marker:
            return
            
        request = SetMarkerPosition.Request()
        request.marker_id = self.current_marker['id']
        request.marker_position = self.current_marker['pose'].position
        
        future = self.marker_client.call_async(request)
        future.add_done_callback(self.marker_response_callback)

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

    def get_next_frontier(self):
        """Get closest unexplored frontier"""
        if not self.position or not self.unexplored_frontiers:
            return None
            
        closest = min(self.unexplored_frontiers,
                     key=lambda f: math.sqrt((f[0]-self.position.x)**2 + 
                                           (f[1]-self.position.y)**2))
        self.unexplored_frontiers.remove(closest)
        return closest

    def control_loop(self):
        """Main control loop"""
        if not self.scan_data or not self.position:
            return

        msg = Twist()
        
        if self.state == 'verify_marker':
            # Stop and verify marker position
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.report_marker()
            self.state = 'explore'
        
        elif self.state == 'explore':
            if not self.current_frontier:
                self.current_frontier = self.get_next_frontier()
                if not self.current_frontier:
                    # No frontiers, do wall following
                    msg = self.wall_following()
                    
            if self.current_frontier:
                # Move to frontier
                dx = self.current_frontier[0] - self.position.x
                dy = self.current_frontier[1] - self.position.y
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.yaw
                
                # Normalize angle
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                # Check if we've reached the frontier
                distance = math.sqrt(dx*dx + dy*dy)
                if distance < 0.3:
                    self.current_frontier = None
                else:
                    # Move to frontier
                    if abs(angle_diff) > 0.1:
                        msg.angular.z = 0.3 if angle_diff > 0 else -0.3
                    else:
                        msg.linear.x = 0.2
                        msg.angular.z = 0.0
        
        # Add speed limits and obstacle avoidance
        if self.regions['front'] < 0.3:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.direction_modifier
        
        msg.linear.x = max(-0.2, min(0.2, msg.linear.x))
        msg.angular.z = max(-0.5, min(0.5, msg.angular.z))
        
        self.cmd_vel_pub.publish(msg)

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

    def wall_following(self):
        """Basic wall following behavior"""
        msg = Twist()
        
        if self.namespace == 'tb3_0':  # Right wall follower
            if self.regions['front'] > 0.5:  # Front is clear
                msg.linear.x = 0.15
                if self.regions['right'] > 0.4:  # Too far from wall
                    msg.angular.z = -0.2
                elif self.regions['right'] < 0.3:  # Too close to wall
                    msg.angular.z = 0.2
                else:
                    msg.angular.z = 0.0
            else:  # Front obstacle
                msg.linear.x = 0.0
                msg.angular.z = 0.4  # Turn left
        else:  # Left wall follower
            if self.regions['front'] > 0.5:  # Front is clear
                msg.linear.x = 0.15
                if self.regions['left'] > 0.4:  # Too far from wall
                    msg.angular.z = 0.2
                elif self.regions['left'] < 0.3:  # Too close to wall
                    msg.angular.z = -0.2
                else:
                    msg.angular.z = 0.0
            else:  # Front obstacle
                msg.linear.x = 0.0
                msg.angular.z = -0.4  # Turn right
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
