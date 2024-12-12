import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition
from .exploration_strategy import ExplorationStrategy
from .robot_coordinator import RobotCoordinator
from .marker_handler import MarkerHandler

class SearchRescueController(Node):
    def __init__(self):
        super().__init__('search_rescue_controller')
        
        # Initialize components
        self.exploration = ExplorationStrategy(self)
        self.coordinator = RobotCoordinator(self)
        self.marker_handler = MarkerHandler(self)
        
        # Robot positions
        self.robot_positions = {'tb3_0': None, 'tb3_1': None}
        
        # Create cmd_vel publishers
        self.cmd_vel_pubs = {
            'tb3_0': self.create_publisher(Twist, '/tb3_0/cmd_vel', 10),
            'tb3_1': self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)
        }
        
        # Subscribers
        self.create_subscription(
            Odometry,
            '/tb3_0/odom',
            lambda msg: self.odom_callback('tb3_0', msg),
            10
        )
        self.create_subscription(
            Odometry,
            '/tb3_1/odom',
            lambda msg: self.odom_callback('tb3_1', msg),
            10
        )
        
        # ArUco marker subscribers
        self.create_subscription(
            ArucoMarkers,
            '/tb3_0/aruco_markers',
            lambda msg: self.aruco_callback('tb3_0', msg),
            10
        )
        self.create_subscription(
            ArucoMarkers,
            '/tb3_1/aruco_markers',
            lambda msg: self.aruco_callback('tb3_1', msg),
            10
        )
        
        # Timer for exploration updates and movement
        self.create_timer(0.1, self.control_timer_callback)  # 10Hz control loop
        
        self.get_logger().info('Search & Rescue Controller initialized')
        
        # Initial movement to verify robots are responsive
        self.test_movement()

    def test_movement(self):
        """Send a test movement command to verify robot control"""
        self.get_logger().info('Sending test movement commands...')
        twist = Twist()
        twist.linear.x = 0.1  # Small forward velocity
        
        # Send test movement to both robots
        for robot_id, pub in self.cmd_vel_pubs.items():
            pub.publish(twist)
            self.get_logger().info(f'Sent test movement to {robot_id}')

    def odom_callback(self, robot_id, msg):
        self.robot_positions[robot_id] = msg.pose.pose
        self.get_logger().debug(f'Received odom for {robot_id}: '
                              f'x={msg.pose.pose.position.x:.2f}, '
                              f'y={msg.pose.pose.position.y:.2f}')

    def aruco_callback(self, robot_id, msg):
        if msg.marker_ids:  # Only process if markers are detected
            self.get_logger().info(f'Detected markers for {robot_id}: {msg.marker_ids}')
            for i, marker_id in enumerate(msg.marker_ids):
                marker_pose = msg.poses[i]
                self.marker_handler.handle_marker(
                    robot_id,
                    marker_id,
                    marker_pose
                )

    def control_timer_callback(self):
        """Main control loop"""
        # Update exploration goals and move robots
        if any(pos is None for pos in self.robot_positions.values()):
            self.get_logger().warn('Waiting for robot positions...')
            return
            
        # Update exploration goals and get movement commands
        self.exploration.update_exploration_goals(self.robot_positions)

def main(args=None):
    rclpy.init(args=args)
    controller = SearchRescueController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 