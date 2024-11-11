import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import inf

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.min_front_dist = 0.5
        self.min_side_dist = 0.3
        
    def scan_callback(self, msg):
        twist = Twist()
        
        # Get front, left and right distances
        front = min(min(msg.ranges[0:15]), min(msg.ranges[345:359]))
        left = min(msg.ranges[80:100])
        right = min(msg.ranges[260:280])
        
        if front < self.min_front_dist:
            # Too close to wall in front, turn right
            twist.angular.z = -0.5
        elif left < self.min_side_dist:
            # Too close to left wall, turn right slightly
            twist.linear.x = 0.2
            twist.angular.z = -0.2
        elif left > self.min_side_dist * 1.5:
            # Too far from left wall, turn left slightly
            twist.linear.x = 0.2
            twist.angular.z = 0.2
        else:
            # Follow wall straight
            twist.linear.x = 0.3
            
        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 