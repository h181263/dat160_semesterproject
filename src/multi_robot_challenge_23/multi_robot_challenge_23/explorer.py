import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from math import atan2, sqrt, pi
from tf_transformations import euler_from_quaternion
import numpy as np

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.map_data = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.exploring = True
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.robot_pos_pub = self.create_publisher(Point, '/robot_pos', 10)
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.explore)
        self.target_update_timer = self.create_timer(5.0, self.update_target)
    
    def map_callback(self, msg):
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
        pos_msg = Point()
        pos_msg.x = self.current_x
        pos_msg.y = self.current_y
        self.robot_pos_pub.publish(pos_msg)
    
    def update_target(self):
        if not self.map_data:
            return
            
        # Convert map to numpy array
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_array = np.array(self.map_data.data).reshape(height, width)
        
        # Find unexplored areas (cells with -1 value)
        unexplored = np.where(map_array == -1)
        if len(unexplored[0]) == 0:
            self.exploring = False
            return
            
        # Choose random unexplored point
        idx = np.random.randint(len(unexplored[0]))
        map_x = unexplored[1][idx]
        map_y = unexplored[0][idx]
        
        # Convert to world coordinates
        self.target_x = map_x * self.map_resolution + self.map_data.info.origin.position.x
        self.target_y = map_y * self.map_resolution + self.map_data.info.origin.position.y
        
        self.get_logger().info(f'New target: ({self.target_x}, {self.target_y})')
    
    def explore(self):
        if not hasattr(self, 'current_yaw') or not self.exploring:
            return
            
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = sqrt(dx**2 + dy**2)
        target_angle = atan2(dy, dx)
        
        angle_diff = target_angle - self.current_yaw
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi
        
        cmd = Twist()
        if distance > 0.2:
            if abs(angle_diff) > 0.1:
                cmd.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                cmd.linear.x = min(0.2, distance)
                cmd.angular.z = angle_diff
        
        self.cmd_vel_pub.publish(cmd)

def main():
    rclpy.init()
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 