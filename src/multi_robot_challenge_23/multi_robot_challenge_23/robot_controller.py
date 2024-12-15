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
        # Initialize with a default namespace first
        super().__init__('robot_controller')
        
        # Get the actual namespace from ROS
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:  # If empty, use default
            self.namespace = 'tb3_0'
        
        self.get_logger().info(f"Initializing controller for {self.namespace}")
        
        # Publishers - using absolute topic names
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.namespace}/cmd_vel', 10)
        self.fire_pub = self.create_publisher(
            String, '/big_fire_location', 10)
            
        # Subscribers - using absolute topic names
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.namespace}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{self.namespace}/scan', self.scan_callback, 10)
        self.fire_sub = self.create_subscription(
            String, '/big_fire_location', self.fire_callback, 10)
        self.marker_pose_sub = self.create_subscription(
            Pose, f'/{self.namespace}/marker_map_pose', self.marker_pose_callback, 10)
        self.marker_id_sub = self.create_subscription(
            Int64, f'/{self.namespace}/marker_id', self.marker_id_callback, 10)
            
        # Add debug messages
        self.get_logger().info(f"Publishing to cmd_vel: /{self.namespace}/cmd_vel")
        self.get_logger().info(f"Subscribing to odom: /{self.namespace}/odom")
        self.get_logger().info(f"Subscribing to scan: /{self.namespace}/scan")
        
        # Test movement
        self.create_timer(0.1, self.test_movement)  # Add a test movement timer
        
        # Service client for marker reporting
        self.marker_client = self.create_client(
            SetMarkerPosition, 'set_marker_position')
            
        # Robot state
        self.position = None
        self.orientation = 0.0
        self.scan_data = None
        self.current_marker_pose = None
        self.current_marker_id = None
        self.new_marker_detected = False
        self.is_exploring = False
        self.stuck_counter = 0
        self.last_position = Point()
        self.recovery_mode = False
        self.recovery_start_time = None
        
        # Map boundaries (from map yaml files)
        self.map_bounds = {
            'x_min': -10.0,
            'x_max': 10.0,
            'y_min': -10.0,
            'y_max': 10.0
        }
        
        # Movement parameters
        self.linear_speed = 0.12  # Reduced speed for better control
        self.angular_speed = 0.5
        self.min_front_distance = 0.5
        self.stuck_threshold = 0.01
        self.stuck_time_threshold = 10  # Reduced for faster response
        
    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        # Convert quaternion to Euler angles
        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        
    def fire_callback(self, msg):
        fire_data = json.loads(msg.data)
        self.big_fire_location = Point()
        self.big_fire_location.x = fire_data['x']
        self.big_fire_location.y = fire_data['y']
        
    def report_marker(self, marker_id, position):
        request = SetMarkerPosition.Request()
        request.marker_id = marker_id
        request.marker_position = position
        
        future = self.marker_client.call_async(request)
        return future
        
    def get_next_frontier(self):
        if not self.scan_data:
            return None
            
        # Convert laser scan to points
        angles = np.linspace(-math.pi/2, math.pi/2, len(self.scan_data))
        points = []
        
        for i, distance in enumerate(self.scan_data):
            if distance < float('inf'):
                angle = angles[i] + self.orientation
                x = self.position.x + distance * math.cos(angle)
                y = self.position.y + distance * math.sin(angle)
                points.append((x, y))
                
        # Find gaps in the scan that could be frontiers
        frontier_points = []
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            if dist > 1.0:  # Gap threshold
                frontier_x = (p1[0] + p2[0]) / 2
                frontier_y = (p1[1] + p2[1]) / 2
                
                # Check if point is within map bounds and not visited
                if (self.map_bounds['x_min'] <= frontier_x <= self.map_bounds['x_max'] and
                    self.map_bounds['y_min'] <= frontier_y <= self.map_bounds['y_max']):
                    
                    # Check if point is not too close to visited points
                    is_new = True
                    for visited in self.visited_points:
                        if math.sqrt((frontier_x - visited[0])**2 + 
                                   (frontier_y - visited[1])**2) < 1.0:
                            is_new = False
                            break
                            
                    if is_new:
                        frontier_points.append((frontier_x, frontier_y))
        
        if frontier_points:
            # Choose the closest frontier point
            closest_frontier = min(frontier_points, 
                key=lambda p: (p[0] - self.position.x)**2 + 
                            (p[1] - self.position.y)**2)
            
            target = Point()
            target.x = closest_frontier[0]
            target.y = closest_frontier[1]
            
            self.visited_points.append(closest_frontier)
            return target
            
        return None
        
    def at_target(self, target):
        if not target:
            return True
            
        distance = math.sqrt(
            (target.x - self.position.x)**2 + 
            (target.y - self.position.y)**2)
            
        return distance < self.distance_threshold
        
    def rotate_to_heading(self, target_heading):
        while True:
            # Calculate angle difference in range [-pi, pi]
            angle_diff = (target_heading - self.orientation + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_diff) < self.angle_threshold:
                break
                
            # Create rotation command
            twist = Twist()
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            
            # Small delay to prevent CPU overload
            rclpy.spin_once(self, timeout_sec=0.1)
            
    def move_forward_safely(self):
        if not self.scan_data:
            return
            
        # Check for obstacles in front
        front_scan = min(self.scan_data[len(self.scan_data)//3:2*len(self.scan_data)//3])
        
        twist = Twist()
        if front_scan > 0.5:  # Safe distance threshold
            twist.linear.x = self.linear_speed
        else:
            twist.linear.x = 0.0
            
        self.cmd_vel_pub.publish(twist)
        
    def detect_marker(self):
        # This would be implemented using the camera feed and ArUco detection
        # For now, return False as placeholder
        return False
        
    def get_marker_info(self):
        # This would return the detected marker's ID and position
        # For now, return placeholder values
        return None, None
        
    def coordinate_big_fire(self, position):
        # Publish fire location for other robot
        fire_msg = String()
        fire_msg.data = json.dumps({
            'x': position.x,
            'y': position.y
        })
        self.fire_pub.publish(fire_msg)
        
        # Move to fire location
        self.move_to_point(position)
        
        # Wait at location
        while rclpy.ok():
            # Keep position but look for other robot
            self.rotate_to_heading(self.orientation + math.pi/4)
            rclpy.spin_once(self, timeout_sec=0.1)

    def marker_pose_callback(self, msg):
        """Callback for receiving marker poses"""
        self.current_marker_pose = msg
        self.new_marker_detected = True
        self.get_logger().info(f"{self.namespace} detected marker at position: x={msg.position.x:.2f}, y={msg.position.y:.2f}")

    def marker_id_callback(self, msg):
        """Callback for receiving marker IDs"""
        self.current_marker_id = msg.data
        self.get_logger().info(f"{self.namespace} detected marker ID: {msg.data}")

    def exploration_callback(self):
        """Main control loop"""
        if not self.scan_data or self.position is None:
            self.get_logger().warn(f"{self.namespace}: Waiting for sensor data...")
            return

        twist = Twist()
        
        # Convert inf readings to max range and get scan data
        ranges = [min(x, 10.0) if not math.isinf(x) else 10.0 for x in self.scan_data]
        
        # Get critical distances
        front_center = min(ranges[175:185])
        front_left = min(ranges[150:175])
        front_right = min(ranges[185:210])
        left = min(ranges[60:120])
        right = min(ranges[240:300])
        
        # Debug logging
        self.get_logger().info(
            f"{self.namespace} distances - Front: {front_center:.2f}, "
            f"FL: {front_left:.2f}, FR: {front_right:.2f}, "
            f"L: {left:.2f}, R: {right:.2f}"
        )

        # STOP if about to hit something
        if front_center < 0.3 or front_left < 0.3 or front_right < 0.3:
            self.get_logger().warn(f"{self.namespace}: EMERGENCY STOP!")
            
            # Aggressive backup and turn
            twist.linear.x = -0.2
            if left > right:
                twist.angular.z = 1.5  # Sharp left turn
            else:
                twist.angular.z = -1.5  # Sharp right turn
                
            self.cmd_vel_pub.publish(twist)
            return

        # Check if we're too close to any wall
        too_close = front_center < 0.5 or front_left < 0.4 or front_right < 0.4
        
        if too_close:
            self.get_logger().info(f"{self.namespace}: Too close to wall, avoiding...")
            # Stop and turn
            twist.linear.x = 0.0
            
            # Determine escape direction
            if left > right:
                twist.angular.z = 1.0  # Turn left
            else:
                twist.angular.z = -1.0  # Turn right
                
        else:
            # Normal exploration mode
            twist.linear.x = 0.1  # Very conservative forward speed
            
            # Active wall avoidance
            if left < 0.6:  # Increased safety margin
                twist.angular.z = -0.5  # Turn right
            elif right < 0.6:  # Increased safety margin
                twist.angular.z = 0.5  # Turn left
            else:
                # Random wandering with bias
                if np.random.random() < 0.1:  # 10% chance to change direction
                    twist.angular.z = (np.random.random() - 0.5) * 0.5
        
        # Additional stuck detection
        if self.check_if_stuck():
            self.get_logger().warn(f"{self.namespace}: Stuck detected!")
            # Very aggressive recovery
            twist.linear.x = -0.2
            twist.angular.z = 2.0 if np.random.random() > 0.5 else -2.0
            self.stuck_counter = 0  # Reset stuck counter
        
        # Ensure we're not exceeding speed limits
        twist.linear.x = max(-0.2, min(0.2, twist.linear.x))
        twist.angular.z = max(-2.0, min(2.0, twist.angular.z))
        
        # Debug movement commands
        self.get_logger().info(
            f"{self.namespace} movement - Linear: {twist.linear.x:.2f}, "
            f"Angular: {twist.angular.z:.2f}"
        )
        
        self.cmd_vel_pub.publish(twist)

    def check_if_stuck(self):
        """More sensitive stuck detection"""
        if not hasattr(self.last_position, 'x'):
            self.last_position = self.position
            return False

        distance_moved = math.sqrt(
            (self.position.x - self.last_position.x) ** 2 +
            (self.position.y - self.last_position.y) ** 2
        )

        # Update last position
        self.last_position = Point()
        self.last_position.x = self.position.x
        self.last_position.y = self.position.y

        # More sensitive stuck detection
        if distance_moved < 0.01:  # Smaller threshold
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        return self.stuck_counter > 5  # Faster response

    def test_movement(self):
        """Test function to verify basic movement"""
        twist = Twist()
        twist.linear.x = 0.1  # Try to move forward slowly
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"{self.namespace}: Testing movement...")

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
