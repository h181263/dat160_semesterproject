import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_aruco_interfaces.msg import ArucoMarkers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')
        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.score_pub = self.create_publisher(String, 'score', 10)
        self.detected_fires = set()
        self.robot_count = 2  # Starting with 2 robots
        
    def aruco_callback(self, msg):
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id not in self.detected_fires:
                self.detected_fires.add(marker_id)
                
                # Report to scoring system
                score_msg = String()
                score_msg.data = f"FIRE_DETECTED_{marker_id}"
                self.score_pub.publish(score_msg)
                
                # If big fire (multiple markers close together), spawn new robots
                if self.is_big_fire(msg, i):
                    self.spawn_new_robots(msg.poses[i])
                    
    def is_big_fire(self, msg, index):
        # Check if there are multiple fire markers close to each other
        marker_pose = msg.poses[index]
        nearby_markers = 0
        
        for i, pose in enumerate(msg.poses):
            if i != index:
                dist = ((pose.position.x - marker_pose.position.x)**2 + 
                       (pose.position.y - marker_pose.position.y)**2)**0.5
                if dist < 1.0:  # Within 1 meter
                    nearby_markers += 1
                    
        return nearby_markers >= 2
    
    def spawn_new_robots(self, fire_pose):
        if self.robot_count >= 4:  # Maximum 4 robots
            return
            
        package_name = 'multi_robot_challenge_23'
        spawn_file = os.path.join(
            get_package_share_directory(package_name),
            'launch',
            'spawn_robot.launch.py'
        )
        
        for i in range(2):  # Spawn 2 new robots
            robot_name = f'tb3_{self.robot_count}'
            self.robot_count += 1
            
            ld = LaunchDescription([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_file),
                    launch_arguments={
                        'namespace': robot_name,
                        'x': str(fire_pose.position.x + i),
                        'y': str(fire_pose.position.y),
                        'yaw': '0.0'
                    }.items()
                )
            ])
            ld.launch()

def main():
    rclpy.init()
    fire_detector = FireDetector()
    rclpy.spin(fire_detector)
    fire_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 