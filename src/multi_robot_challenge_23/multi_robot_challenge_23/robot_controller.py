import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from scoring_interfaces.srv import SetMarkerPosition
from tf_transformations import euler_from_quaternion
import math
import numpy as np
from std_msgs.msg import String
import json
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64

class RobotController(Node):
    def __init__(self, namespace):
        super().__init__(f'{namespace}_controller')
        self.namespace = namespace
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{namespace}/cmd_vel', 10)
        self.fire_pub = self.create_publisher(
            String, '/big_fire_location', 10)
            
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, f'/{namespace}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{namespace}/scan', self.scan_callback, 10)
        self.fire_sub = self.create_subscription(
            String, '/big_fire_location', self.fire_callback, 10)
            
        # Service client for marker reporting
        self.marker_client = self.create_client(
            SetMarkerPosition, 'set_marker_position')
            
        # Robot state
        self.position = Point()
        self.orientation = 0.0  # yaw in radians
        self.scan_data = None
        self.frontiers = []
        self.visited_points = []
        self.big_fire_location = None
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.distance_threshold = 0.3
        self.angle_threshold = 0.1
        
        # Map boundaries (from map yaml files)
        self.map_bounds = {
            'x_min': -10.0,
            'x_max': 10.0,
            'y_min': -10.0,
            'y_max': 10.0
        }
        
        # Add subscribers for marker detection
        self.marker_pose_sub = self.create_subscription(
            Pose, f'/{namespace}/marker_map_pose', self.marker_pose_callback, 10)
        self.marker_id_sub = self.create_subscription(
            Int64, f'/{namespace}/marker_id', self.marker_id_callback, 10)
        
        # Add marker detection state
        self.current_marker_pose = None
        self.current_marker_id = None
        self.new_marker_detected = False
        
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
        
    def marker_pose_callback(self, msg):
        self.current_marker_pose = msg
        self.new_marker_detected = True

    def marker_id_callback(self, msg):
        self.current_marker_id = msg.data

    def detect_marker(self):
        """Check if a new marker has been detected"""
        if self.new_marker_detected:
            self.new_marker_detected = False
            return True
        return False

    def get_marker_info(self):
        """Return the current marker's ID and position"""
        if self.current_marker_pose and self.current_marker_id is not None:
            marker_position = Point()
            marker_position.x = self.current_marker_pose.position.x
            marker_position.y = self.current_marker_pose.position.y
            marker_position.z = self.current_marker_pose.position.z
            return self.current_marker_id, marker_position
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

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController('tb3_0')  # Change namespace as needed
    
    try:
        controller.explore_environment()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
