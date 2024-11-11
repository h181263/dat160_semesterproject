import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        
        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(Int32, '/marker', 10)
        self.robot_pos_pub = self.create_publisher(Point, '/robot_pos', 10)
        
        self.detected_markers = set()
        self.big_fires_handled = set()
        self.next_robot_id = 2  # Start from tb3_2 for fire response robots
        
    def spawn_fire_response_robot(self, x, y, robot_number):
        package_name = 'multi_robot_challenge_23'
        
        # Spawn the robot using ros2 run
        namespace = f'tb3_{robot_number}'
        spawn_cmd = [
            'ros2', 'launch', package_name, 'spawn_robot.launch.py',
            f'namespace:={namespace}',
            f'x:={x + 1.0}',  # Offset to avoid collision
            f'y:={y + 1.0}',
            'yaw:=0.0'
        ]
        subprocess.Popen(spawn_cmd)
        
        # Launch the fire navigator for this robot
        nav_cmd = [
            'ros2', 'run', package_name, 'fire_navigator',
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '-p', f'target_x:={x}',
            '-p', f'target_y:={y}'
        ]
        subprocess.Popen(nav_cmd)
        
    def is_big_fire(self, marker_id, marker_pose):
        # Check if marker 4 (usually indicates big fire)
        return marker_id == 4
    
    def aruco_callback(self, msg):
        if not msg.marker_ids:
            return
            
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id not in self.detected_markers:
                self.detected_markers.add(marker_id)
                
                # Publish marker ID to scoring system
                marker_msg = Int32()
                marker_msg.data = marker_id
                self.marker_pub.publish(marker_msg)
                
                # Publish robot position
                pos_msg = Point()
                pos_msg.x = msg.poses[i].position.x
                pos_msg.y = msg.poses[i].position.y
                pos_msg.z = 0.0
                self.robot_pos_pub.publish(pos_msg)
                
                # Check for big fire
                if (self.is_big_fire(marker_id, msg.poses[i]) and 
                    marker_id not in self.big_fires_handled):
                    self.big_fires_handled.add(marker_id)
                    
                    # Spawn two fire response robots
                    fire_x = msg.poses[i].position.x
                    fire_y = msg.poses[i].position.y
                    
                    self.get_logger().info(f'Spawning fire response robots for fire at ({fire_x}, {fire_y})')
                    
                    # Spawn two robots
                    self.spawn_fire_response_robot(fire_x, fire_y, self.next_robot_id)
                    self.next_robot_id += 1
                    self.spawn_fire_response_robot(fire_x, fire_y, self.next_robot_id)
                    self.next_robot_id += 1

def main():
    rclpy.init()
    marker_detector = MarkerDetector()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 