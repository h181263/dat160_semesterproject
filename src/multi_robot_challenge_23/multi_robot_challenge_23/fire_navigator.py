import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import atan2, sqrt
from tf_transformations import euler_from_quaternion

class FireNavigator(Node):
    def __init__(self):
        super().__init__('fire_navigator')
        
        # Get target position from parameter
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.robot_pos_pub = self.create_publisher(Point, '/robot_pos', 10)
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.navigate)
        
        self.get_logger().info(f'Navigating to fire at ({self.target_x}, {self.target_y})')
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
        # Publish robot position for scoring system
        pos_msg = Point()
        pos_msg.x = self.current_x
        pos_msg.y = self.current_y
        self.robot_pos_pub.publish(pos_msg)
        
    def navigate(self):
        if not hasattr(self, 'current_yaw'):
            return
            
        # Calculate distance and angle to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = sqrt(dx**2 + dy**2)
        target_angle = atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.current_yaw
        if angle_diff > 3.14159:
            angle_diff -= 2 * 3.14159
        elif angle_diff < -3.14159:
            angle_diff += 2 * 3.14159
        
        # Create and publish velocity command
        cmd = Twist()
        if distance > 0.2:  # If not at target
            if abs(angle_diff) > 0.1:  # First align with target
                cmd.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:  # Then move towards it
                cmd.linear.x = min(0.2, distance)
                cmd.angular.z = angle_diff
        else:
            self.get_logger().info('Reached fire location!')
            
        self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    navigator = FireNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 