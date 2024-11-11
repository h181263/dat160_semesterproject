import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import inf

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        # Get robot namespace to determine behavior
        self.namespace = self.get_namespace()
        self.get_logger().info(f'Starting wall follower for {self.namespace}')
        
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Different parameters for inner/outer wall following
        if 'tb3_0' in self.namespace:  # Outer wall follower
            self.min_front_dist = 0.5
            self.min_side_dist = 0.4
            self.follow_left_wall = True  # Follow left wall for outer perimeter
        else:  # Inner wall follower
            self.min_front_dist = 0.5
            self.min_side_dist = 0.3
            self.follow_left_wall = False  # Follow right wall for inner perimeter
        
    def scan_callback(self, msg):
        twist = Twist()
        
        # Get front, left and right distances
        front = min(min(msg.ranges[0:15]), min(msg.ranges[345:359]))
        left = min(msg.ranges[80:100])
        right = min(msg.ranges[260:280])
        
        if self.follow_left_wall:
            self.follow_wall(twist, front, left, 'left')
        else:
            self.follow_wall(twist, front, right, 'right')
            
        self.cmd_vel_pub.publish(twist)
    
    def follow_wall(self, twist, front, side_dist, side):
        if front < self.min_front_dist:
            # Too close to wall in front, turn away from wall
            twist.angular.z = -0.5 if side == 'left' else 0.5
            twist.linear.x = 0.0
        elif side_dist < self.min_side_dist:
            # Too close to side wall, turn slightly away
            twist.linear.x = 0.2
            twist.angular.z = -0.2 if side == 'left' else 0.2
        elif side_dist > self.min_side_dist * 1.5:
            # Too far from side wall, turn towards it
            twist.linear.x = 0.2
            twist.angular.z = 0.2 if side == 'left' else -0.2
        else:
            # Follow wall straight
            twist.linear.x = 0.3
            twist.angular.z = 0.0

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 