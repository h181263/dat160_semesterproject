import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from scoring_interfaces.srv import SetMarkerPosition
from std_msgs.msg import String, Int64
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import json

class RobotController(Node):
    def __init__(self):
        # Get namespace from node name
        namespace = rclpy.get_namespace().strip('/')
        super().__init__(f'{namespace}_controller')
        
        self.namespace = namespace
        self.get_logger().info(f"Initializing controller for {namespace}")
        
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
        self.marker_pose_sub = self.create_subscription(
            Pose, f'/{namespace}/marker_map_pose', self.marker_pose_callback, 10)
        self.marker_id_sub = self.create_subscription(
            Int64, f'/{namespace}/marker_id', self.marker_id_callback, 10)
            
        # Service client for marker reporting
        self.marker_client = self.create_client(
            SetMarkerPosition, 'set_marker_position')
            
        # Robot state
        self.position = Point()
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
        
        # Movement parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.min_front_distance = 0.5
        self.stuck_threshold = 0.01  # Minimum movement to not be considered stuck
        self.stuck_time_threshold = 20  # Number of iterations before considering stuck
        
        # Create a timer for exploration
        self.create_timer(0.1, self.exploration_callback)  # 10Hz update rate

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.orientation = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    def marker_pose_callback(self, msg):
        self.current_marker_pose = msg
        self.new_marker_detected = True

    def marker_id_callback(self, msg):
        self.current_marker_id = msg.data

    def fire_callback(self, msg):
        """Handle big fire coordination between robots"""
        if not self.current_marker_id == 4:  # Only handle if we're not already at fire
            try:
                fire_data = json.loads(msg.data)
                fire_position = Point()
                fire_position.x = fire_data['x']
                fire_position.y = fire_data['y']
                
                # Calculate distance to fire
                distance_to_fire = math.sqrt(
                    (self.position.x - fire_position.x) ** 2 +
                    (self.position.y - fire_position.y) ** 2
                )
                
                # If within range of fire
                if distance_to_fire < 2.0:
                    # Similar to leader.py's lidar value check
                    if self.scan_data:
                        front_distance = self.scan_data[180]  # Center reading
                        if front_distance < 2.0:
                            self.get_logger().info(f"{self.namespace} detected another robot near fire")
                            # Report marker 4 (big fire) when both robots are present
                            self.report_marker(4, fire_position)
                else:
                    # Move towards fire location
                    self.get_logger().info(f"{self.namespace} moving to fire location")
                    twist = Twist()
                    
                    # Calculate angle to fire
                    angle_to_fire = math.atan2(
                        fire_position.y - self.position.y,
                        fire_position.x - self.position.x
                    )
                    
                    # Rotate and move towards fire
                    angle_diff = angle_to_fire - self.orientation
                    if abs(angle_diff) > 0.1:
                        twist.angular.z = 0.3 if angle_diff > 0 else -0.3
                    else:
                        twist.linear.x = 0.2
                    
                    self.cmd_vel_pub.publish(twist)
                    
            except json.JSONDecodeError:
                self.get_logger().error("Failed to parse fire location message")

    def check_if_stuck(self):
        if not hasattr(self.last_position, 'x'):
            self.last_position = self.position
            return False

        distance_moved = math.sqrt(
            (self.position.x - self.last_position.x) ** 2 +
            (self.position.y - self.last_position.y) ** 2
        )

        if distance_moved < self.stuck_threshold:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            self.recovery_mode = False

        self.last_position = self.position
        return self.stuck_counter > self.stuck_time_threshold

    def get_safe_direction(self):
        if not self.scan_data:
            return 0.0

        # Convert inf readings to max range
        ranges = [min(x, 10.0) if not math.isinf(x) else 10.0 for x in self.scan_data]
        
        # Find the longest range sector
        sector_size = 30
        num_sectors = len(ranges) // sector_size
        sector_ranges = []
        
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            sector_ranges.append(sum(ranges[start_idx:end_idx]) / sector_size)
        
        best_sector = sector_ranges.index(max(sector_ranges))
        return (best_sector - num_sectors//2) * sector_size * (math.pi / 180.0)

    def exploration_callback(self):
        """Main control loop"""
        if not self.scan_data or self.position is None:
            return  # Wait for sensor data
            
        if not self.is_exploring:
            self.is_exploring = True
            self.get_logger().info(f"{self.namespace} starting exploration")
        
        twist = Twist()
        
        # Check if stuck
        if self.check_if_stuck():
            if not self.recovery_mode:
                self.recovery_mode = True
                self.get_logger().info(f"{self.namespace} is stuck, entering recovery mode")
            
            # Recovery behavior
            safe_direction = self.get_safe_direction()
            twist.linear.x = -0.1  # Back up slowly
            twist.angular.z = self.angular_speed * (1 if safe_direction > 0 else -1)
            
        else:
            # Normal exploration
            if self.scan_data:
                # Check front arc for obstacles
                front_arc = self.scan_data[-45:] + self.scan_data[:45]
                min_front_distance = min([x for x in front_arc if not math.isinf(x)], default=10)
                
                # Check side arcs for wall following
                left_arc = min([x for x in self.scan_data[60:120] if not math.isinf(x)], default=10)
                right_arc = min([x for x in self.scan_data[-120:-60] if not math.isinf(x)], default=10)
                
                if min_front_distance > self.min_front_distance:
                    twist.linear.x = self.linear_speed
                    # Wall following behavior
                    if left_arc < 1.0:  # If wall on left
                        twist.angular.z = -0.2  # Turn slightly right
                    elif right_arc < 1.0:  # If wall on right
                        twist.angular.z = 0.2  # Turn slightly left
                else:
                    # Find best direction to turn
                    safe_direction = self.get_safe_direction()
                    twist.angular.z = self.angular_speed * (1 if safe_direction > 0 else -1)
        
        self.cmd_vel_pub.publish(twist)
        
        # Check for markers
        if self.detect_marker():
            marker_id, position = self.get_marker_info()
            if marker_id is not None and position is not None:
                self.report_marker(marker_id, position)
                self.get_logger().info(f"Reported marker {marker_id} at position {position}")

    def detect_marker(self):
        if self.new_marker_detected:
            self.new_marker_detected = False
            return True
        return False

    def get_marker_info(self):
        if self.current_marker_pose and self.current_marker_id is not None:
            marker_position = Point()
            marker_position.x = self.current_marker_pose.position.x
            marker_position.y = self.current_marker_pose.position.y
            marker_position.z = self.current_marker_pose.position.z
            return self.current_marker_id, marker_position
        return None, None

    def report_marker(self, marker_id, position):
        request = SetMarkerPosition.Request()
        request.marker_id = marker_id
        request.marker_position = position
        
        future = self.marker_client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
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