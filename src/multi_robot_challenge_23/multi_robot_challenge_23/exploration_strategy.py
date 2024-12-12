import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid

class ExplorationStrategy:
    def __init__(self, node):
        self.node = node
        self.exploration_points = []
        self.assigned_points = {'tb3_0': [], 'tb3_1': []}
        
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

    def map_callback(self, msg):
        # Update exploration points based on map
        self.generate_exploration_points(msg)

    def generate_exploration_points(self, map_msg):
        # Generate exploration points based on map
        points = []
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height
        
        # Generate points in a grid pattern
        for i in range(0, width, 10):
            for j in range(0, height, 10):
                index = j * width + i
                if map_msg.data[index] == 0:  # Free space
                    x = i * resolution + map_msg.info.origin.position.x
                    y = j * resolution + map_msg.info.origin.position.y
                    points.append(Point(x=x, y=y, z=0.0))
        
        self.exploration_points = points

    def update_exploration_goals(self, robot_positions):
        # Assign new exploration points to robots
        for robot_id, current_pos in robot_positions.items():
            if not self.assigned_points[robot_id]:
                # Find nearest unexplored point
                nearest_point = self.find_nearest_point(
                    current_pos,
                    self.exploration_points
                )
                if nearest_point:
                    self.assigned_points[robot_id] = nearest_point
                    self.exploration_points.remove(nearest_point)

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