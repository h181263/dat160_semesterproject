from geometry_msgs.msg import Pose
import numpy as np

class RobotCoordinator:
    def __init__(self, node):
        self.node = node
        self.robot_positions = {}
        self.big_fire_position = None
        self.robots_at_fire = set()
        
        # Distance threshold for considering robots "at" the fire
        self.fire_threshold = 2.0  # meters

    def update_robot_position(self, robot_id, pose):
        self.robot_positions[robot_id] = pose
        self.check_big_fire_status()

    def set_big_fire_position(self, position):
        self.big_fire_position = position
        self.check_big_fire_status()

    def check_big_fire_status(self):
        if not self.big_fire_position:
            return
            
        for robot_id, pose in self.robot_positions.items():
            distance = self.calculate_distance(
                pose.position,
                self.big_fire_position
            )
            
            if distance < self.fire_threshold:
                self.robots_at_fire.add(robot_id)
            else:
                self.robots_at_fire.discard(robot_id)
                
        if len(self.robots_at_fire) >= 2:
            self.node.get_logger().info('Two robots at big fire!')
            # Additional handling for big fire extinguishing

    def calculate_distance(self, pos1, pos2):
        return np.sqrt(
            (pos1.x - pos2.x)**2 +
            (pos1.y - pos2.y)**2
        )

    def coordinate_big_fire_response(self, fire_position):
        self.big_fire_position = fire_position
        # Implement coordination strategy for getting both robots to the fire 