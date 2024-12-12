import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
import math

class ExplorationStrategy:
    def __init__(self, node):
        self.node = node
        self.exploration_points = []
        self.assigned_points = {'tb3_0': None, 'tb3_1': None}
        self.current_goals = {'tb3_0': None, 'tb3_1': None}
        
        # Publishers for robot movement
        self.cmd_vel_pubs = {
            'tb3_0': node.create_publisher(Twist, '/tb3_0/cmd_vel', 10),
            'tb3_1': node.create_publisher(Twist, '/tb3_1/cmd_vel', 10)
        }
        
        # Subscribe to map updates
        node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Control parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.position_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians

    def update_exploration_goals(self, robot_positions):
        for robot_id, current_pos in robot_positions.items():
            if current_pos is None:
                continue

            # If no assigned point or reached current goal, get new point
            if (self.assigned_points[robot_id] is None or 
                self.is_point_reached(current_pos, self.assigned_points[robot_id])):
                
                nearest_point = self.find_nearest_point(
                    current_pos,
                    self.exploration_points
                )
                if nearest_point:
                    self.assigned_points[robot_id] = nearest_point
                    self.exploration_points.remove(nearest_point)
                    self.node.get_logger().info(f'New goal for {robot_id}: {nearest_point}')

            # Move robot towards its goal
            if self.assigned_points[robot_id]:
                self.move_robot_to_point(
                    robot_id,
                    current_pos,
                    self.assigned_points[robot_id]
                )

    def move_robot_to_point(self, robot_id, current_pos, target_point):
        # Calculate angle to target
        dx = target_point.x - current_pos.position.x
        dy = target_point.y - current_pos.position.y
        target_angle = math.atan2(dy, dx)

        # Get current orientation (assuming quaternion)
        current_angle = self.get_yaw_from_pose(current_pos)

        # Calculate angle difference
        angle_diff = self.normalize_angle(target_angle - current_angle)

        # Create Twist message
        cmd_vel = Twist()

        # If not facing the target, rotate first
        if abs(angle_diff) > self.angle_tolerance:
            cmd_vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            # If facing the target, move forward
            distance = self.calculate_distance(current_pos, target_point)
            if distance > self.position_tolerance:
                cmd_vel.linear.x = self.linear_speed
                # Small angular correction while moving
                cmd_vel.angular.z = angle_diff

        # Publish movement command
        self.cmd_vel_pubs[robot_id].publish(cmd_vel)

    def get_yaw_from_pose(self, pose):
        # Extract yaw from quaternion
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def is_point_reached(self, current_pos, target_point):
        distance = self.calculate_distance(current_pos, target_point)
        return distance < self.position_tolerance

    def map_callback(self, msg):
        if not self.exploration_points:  # Only generate points if we don't have any
            self.generate_exploration_points(msg)

    def generate_exploration_points(self, map_msg):
        points = []
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height
        
        # Generate points in a grid pattern with larger spacing
        step = 20  # Increase step size for fewer points
        for i in range(0, width, step):
            for j in range(0, height, step):
                index = j * width + i
                if 0 <= index < len(map_msg.data) and map_msg.data[index] == 0:  # Free space
                    x = i * resolution + map_msg.info.origin.position.x
                    y = j * resolution + map_msg.info.origin.position.y
                    points.append(Point(x=x, y=y, z=0.0))
        
        self.exploration_points = points
        self.node.get_logger().info(f'Generated {len(points)} exploration points')

    def find_nearest_point(self, current_pos, points):
        if not points:
            return None
            
        distances = [
            self.calculate_distance(current_pos, point)
            for point in points
        ]
        return points[np.argmin(distances)]

    def calculate_distance(self, pos1, pos2):
        return np.sqrt(
            (pos1.position.x - pos2.x)**2 +
            (pos1.position.y - pos2.y)**2
        )