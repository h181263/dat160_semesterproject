import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from bug2_interfaces.srv import GoToPoint
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        # Clients for le scripts
        self.wall_follower_client = self.create_client(SetBool, 'wall_follower_service')
        self.go_to_point_client = self.create_client(GoToPoint, 'go_to_point_service')

        # Subscribing to laserscan to detect obstacles
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser,10)

        # Current position
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)

        # Halp stopping le robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.initial_distance = float('inf')
        self.current_position = Point()
    
        self.state = "go_to_point" # Starter i goToPoint
        self.target_position = Point(x = 3.0, y = 6.00, z = 0.0) # Setter mål posisjon
        self.goal_tolerance = 0.05 # How close need to be to be like, yay
        self.goal_reached = False 

        # If services are available
        while not self.wall_follower_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for le wall follower service')
        while not self.go_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for le go to point service')

        # Start with go to point mode
        self.switch_to_go_to_point()

    def clbk_odom(self, msg):
         self.current_position = msg.pose.pose.position
              

         #Is goal reached
         if self.is_goal_reached() and not self.goal_reached:
              self.get_logger().info("Goal reached, yaaaaay das is so darn awzm")
              self.stop_robot()
              self.goal_reached = True
              

    def is_goal_reached(self):
         distance = self.distance_to_target()
         return distance <= self.goal_tolerance
    
    def clbk_laser(self, msg):
        front_range = min(min(msg.ranges[0:30]), min(msg.ranges[330:359]))
        # Hvis i go to point modus
        if self.state == "go_to_point":
            if front_range < 0.5:
                self.get_logger().info("Obstacle ahead, time to switch to wall follower")
                self.switch_to_wall_follower()
        elif self.state == "wall_follower":
            #Check to see if target is closer, than when wall follower started
            current_distance = self.distance_to_target()
            if current_distance < self.initial_distance:
                self.get_logger().info("Is closer to target, time to switch back to go_to_pooint")
                self.switch_to_go_to_point()

    def distance_to_target(self):
        # Calculate distance to target from current position
        return math.sqrt((self.target_position.x - self.current_position.x)** 2 +
                         (self.target_position.y - self.current_position.y)** 2)

    
    # To avoid both of them being active at the same time, start by deactivating "the other" before starting. 
    def switch_to_go_to_point(self):
        self.get_logger().info('Switching to go to point mode')
        self.state = "go_to_point"

        # Deactivate wall follower first
        wall_follower_req = SetBool.Request()
        wall_follower_req.data = False
        self.wall_follower_client.call_async(wall_follower_req)

        # Activate go-to-point
        request = GoToPoint.Request()
        request.move_switch = True
        request.target_position = self.target_position
        self.go_to_point_client.call_async(request)

    def switch_to_wall_follower(self):
        self.get_logger().info('Switching to wall follower mode')
        self.state = "wall_follower"
        self.initial_distance = self.distance_to_target()

        # Deactivate go-to-point first
        go_to_point_req = GoToPoint.Request()
        go_to_point_req.move_switch = False
        self.go_to_point_client.call_async(go_to_point_req)
       
        # Activate wall follower
        #self.initial_distance = self.distance_to_target()
        request = SetBool.Request()
        request.data = True
        self.wall_follower_client.call_async(request)

    def stop_robot(self):
        # Disable other publishers
        wall_follower_req = SetBool.Request()
        wall_follower_req.data = False
        wall_follower_future = self.wall_follower_client.call_async(wall_follower_req)
        rclpy.spin_until_future_complete(self, wall_follower_future)
        #self.wall_follower_client.call_async(wall_follower_req)

        go_to_point_req = GoToPoint.Request()
        go_to_point_req.move_switch = False
        go_to_point_future = self.go_to_point_client.call_async(go_to_point_req)
        rclpy.spin_until_future_complete(self, go_to_point_future)
        #self.go_to_point_client.call_async(go_to_point_req)


        # Stopping commands
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        # Stop logger was kind of ignored, so added some stuff to see if worked
        # It did not though, sooo, but it is only the logger message, the Goal reached msg is executed so I'll call it quits. 
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_msg)
            rclpy.sleep(0,1)
        self.get_logger().info("Robot staphed")
        
def main(args=None):
        rclpy.init(args=args)
        node = Bug2Controller()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__== '__main__':
        main()