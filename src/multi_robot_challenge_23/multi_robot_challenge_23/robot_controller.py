import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
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
        
        # Timer for exploration updates
        self.create_timer(1.0, self.exploration_timer_callback)
        
        self.get_logger().info('Search & Rescue Controller initialized')

    def odom_callback(self, robot_id, msg):
        self.robot_positions[robot_id] = msg.pose.pose
        self.coordinator.update_robot_position(robot_id, msg.pose.pose)

    def aruco_callback(self, robot_id, msg):
        for i, marker_id in enumerate(msg.marker_ids):
            marker_pose = msg.poses[i]
            self.marker_handler.handle_marker(
                robot_id,
                marker_id,
                marker_pose
            )

    def exploration_timer_callback(self):
        # Update exploration goals based on current state
        self.exploration.update_exploration_goals(self.robot_positions)

def main(args=None):
    rclpy.init(args=args)
    controller = SearchRescueController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 