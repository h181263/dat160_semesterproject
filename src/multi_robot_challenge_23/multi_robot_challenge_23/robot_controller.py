import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64, String
from scoring_interfaces.srv import SetMarkerPosition
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

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
        
        # Subscribe to filtered map
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
        
        # Exploration tracking
        self.unexplored_frontiers = []
        self.current_frontier = None
        self.visited_cells = set()
        
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

    def map_callback(self, msg):
        """Process filtered map data"""
        self.filtered_map = msg
        if not self.unexplored_frontiers:
            self.find_frontiers()

    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates"""
        if not self.filtered_map:
            return None
        
        map_x = int((x - self.filtered_map.info.origin.position.x) / self.filtered_map.info.resolution)
        map_y = int((y - self.filtered_map.info.origin.position.y) / self.filtered_map.info.resolution)
        
        if 0 <= map_x < self.filtered_map.info.width and 0 <= map_y < self.filtered_map.info.height:
            return (map_x, map_y)
        return None

    def map_to_world(self, map_x, map_y):
        """Convert map coordinates to world coordinates"""
        if not self.filtered_map:
            return None
        
        world_x = map_x * self.filtered_map.info.resolution + self.filtered_map.info.origin.position.x
        world_y = map_y * self.filtered_map.info.resolution + self.filtered_map.info.origin.position.y
        
        return (world_x, world_y)

    def find_frontiers(self):
        """Find unexplored areas in the filtered map"""
        if not self.filtered_map:
            return
        
        frontiers = []
        width = self.filtered_map.info.width
        height = self.filtered_map.info.height
        
        # Check each cell in the map
        for y in range(height):
            for x in range(width):
                idx = x + y * width
                
                # If cell is free
                if self.filtered_map.data[idx] == 0:
                    # Check if it's next to unexplored area
                    is_frontier = False
                    for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                        new_x, new_y = x + dx, y + dy
                        if 0 <= new_x < width and 0 <= new_y < height:
                            new_idx = new_x + new_y * width
                            if self.filtered_map.data[new_idx] == -1:  # Unexplored
                                is_frontier = True
                                break
                    
                    if is_frontier:
                        world_coords = self.map_to_world(x, y)
                        if world_coords:
                            frontiers.append(world_coords)
        
        # Assign frontiers based on robot ID
        if self.namespace == 'tb3_0':
            # Sort frontiers by x coordinate (right side first)
            self.unexplored_frontiers = sorted(frontiers, key=lambda f: (-f[0], f[1]))
        else:
            # Sort frontiers by x coordinate (left side first)
            self.unexplored_frontiers = sorted(frontiers, key=lambda f: (f[0], f[1]))

    def get_next_frontier(self):
        """Get the next frontier to explore"""
        while self.unexplored_frontiers:
            frontier = self.unexplored_frontiers.pop(0)
            # Check if frontier is still valid (not already explored)
            map_coords = self.world_to_map(frontier[0], frontier[1])
            if map_coords:
                idx = map_coords[0] + map_coords[1] * self.filtered_map.info.width
                if self.filtered_map.data[idx] == 0:  # Still free space
                    return frontier
        return None

    def move_to_frontier(self):
        """Navigate to the current frontier"""
        if not self.current_frontier or not self.position:
            self.current_frontier = self.get_next_frontier()
            if not self.current_frontier:
                return self.wall_following()  # Default to wall following if no frontiers
        
        msg = Twist()
        
        # Calculate angle to frontier
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
            return self.wall_following()
        
        # Move to frontier
        if abs(angle_diff) > 0.1:
            msg.angular.z = 0.3 if angle_diff > 0 else -0.3
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        
        return msg

    def control_loop(self):
        """Main control loop"""
        if not self.scan_data or not self.filtered_map:
            return

        # Update current position in map
        if self.position:
            map_coords = self.world_to_map(self.position.x, self.position.y)
            if map_coords:
                self.visited_cells.add(map_coords)
        
        # Choose behavior
        if self.regions['front'] < 0.3:  # Very close to obstacle
            msg = self.handle_obstacle()
        elif self.filtered_map:  # Use frontier exploration
            msg = self.move_to_frontier()
        else:  # Default to wall following
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
