import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from bug2_interfaces.srv import GoToPoint
from tf_transformations import euler_from_quaternion
import math

class Bug2Navigation(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        self.start_position = Point()
        self.current_position = Point()
        self.goal_position = Point(x=3.0, y=5.0, z=0.0)
        self.yaw = 0.0
        self.state = 'go_to_point' 
        self.m_line_slope = None
        self.m_line_intercept = None
        self.leave_point = None
        self.closest_distance_to_goal = float('inf')
        self.obstacle_ahead = False

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Publishers
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service Clients
        self.go_to_point_client = self.create_client(GoToPoint, 'go_to_point_switch')
        while not self.go_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for go_to_point_switch service...')
        
        self.wall_follower_client = self.create_client(SetBool, 'wall_follower_switch')
        while not self.wall_follower_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for wall_follower_switch service...')

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.initialize_bug2()

    def initialize_bug2(self):
        self.start_position = self.current_position

        delta_x = self.goal_position.x - self.start_position.x
        delta_y = self.goal_position.y - self.start_position.y
        if delta_x != 0:
            self.m_line_slope = delta_y / delta_x
            self.m_line_intercept = self.start_position.y - self.m_line_slope * self.start_position.x
        else:
            self.m_line_slope = None  # Vertical line
            self.m_line_intercept = self.start_position.x  # x = c

        self.get_logger().info('Bug2 Algorithm Initialized.')
        self.switch_to_go_to_point()

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def laser_callback(self, msg):
        # Check for obstacles in front
        front_ranges = list(msg.ranges[0:10]) + list(msg.ranges[-10:])
        front_ranges = [r for r in front_ranges if not math.isinf(r)]
        if front_ranges:
            front_range = min(front_ranges)
        else:
            front_range = float('inf')

        obstacle_threshold = 0.5
        self.obstacle_ahead = front_range < obstacle_threshold

    def switch_to_go_to_point(self):
        # Stop wall follower
        wall_follower_req = SetBool.Request()
        wall_follower_req.data = False
        self.wall_follower_client.call_async(wall_follower_req)

        # Start go-to-point
        go_to_point_req = GoToPoint.Request()
        go_to_point_req.move_switch = True
        go_to_point_req.target_position = self.goal_position
        self.go_to_point_client.call_async(go_to_point_req)

        self.state = 'go_to_point'
        self.get_logger().info('Switched to Go-to-Point mode.')

    def switch_to_wall_following(self):
        # Stop go-to-point
        go_to_point_req = GoToPoint.Request()
        go_to_point_req.move_switch = False
        go_to_point_req.target_position = self.goal_position
        self.go_to_point_client.call_async(go_to_point_req)

        # Start wall follower
        wall_follower_req = SetBool.Request()
        wall_follower_req.data = True
        self.wall_follower_client.call_async(wall_follower_req)

        self.state = 'wall_following'
        self.get_logger().info('Switched to Wall-Following mode.')

    def timer_callback(self):
        # Check if goal is reached
        distance_to_goal = self.euclidean_distance(self.current_position, self.goal_position)
        if distance_to_goal < 0.2: 
            self.get_logger().info('Goal Reached!')
            # Stop all actions
            self.stop_all()
            return

        if self.state == 'go_to_point':
            if self.obstacle_ahead:
                # Record leave point
                self.leave_point = self.current_position
                self.closest_distance_to_goal = self.euclidean_distance(self.current_position, self.goal_position)
                self.switch_to_wall_following()
        elif self.state == 'wall_following':
            # Check if back on m-line at a closer point to goal
            if self.is_on_m_line() and self.is_closer_to_goal():
                self.switch_to_go_to_point()

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def is_on_m_line(self):
        if self.m_line_slope is not None:
            expected_y = self.m_line_slope * self.current_position.x + self.m_line_intercept
            return abs(self.current_position.y - expected_y) < 0.1 
        else:
            # Vertical line case
            return abs(self.current_position.x - self.m_line_intercept) < 0.1

    def is_closer_to_goal(self):
        current_distance = self.euclidean_distance(self.current_position, self.goal_position)
        return current_distance < self.closest_distance_to_goal

    def stop_all(self):
        # Stop go-to-point
        go_to_point_req = GoToPoint.Request()
        go_to_point_req.move_switch = False
        go_to_point_req.target_position = self.goal_position
        self.go_to_point_client.call_async(go_to_point_req)

        # Stop wall follower
        wall_follower_req = SetBool.Request()
        wall_follower_req.data = False
        self.wall_follower_client.call_async(wall_follower_req)

        # Send zero velocity to stop the robot
        stop_twist = Twist()
        self.twist_pub.publish(stop_twist)

def main(args=None):
    rclpy.init(args=args)
    bug2_controller = Bug2Navigation()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
