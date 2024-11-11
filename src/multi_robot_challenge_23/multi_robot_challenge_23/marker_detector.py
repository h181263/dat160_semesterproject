import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from scoring_interfaces.srv import SetMarkerPosition
import subprocess
import os

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        
        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10
        )
        
        # Create service client for marker position reporting
        self.set_marker_client = self.create_client(
            SetMarkerPosition, 
            'set_marker_position'
        )
        
        self.detected_markers = set()
        self.big_fires_handled = set()
        self.next_robot_id = 2
        
    def report_marker(self, marker_id, position):
        request = SetMarkerPosition.Request()
        request.marker_id = marker_id
        request.marker_position = position
        
        # Call service asynchronously
        future = self.set_marker_client.call_async(request)
        future.add_done_callback(self.report_callback)
    
    def report_callback(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info('Marker position accepted by scoring system')
            else:
                self.get_logger().warn('Marker position rejected by scoring system')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
    
    def aruco_callback(self, msg):
        if not msg.marker_ids:
            return
            
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id not in self.detected_markers:
                self.detected_markers.add(marker_id)
                
                # Report marker position to scoring system
                self.report_marker(marker_id, msg.poses[i].position)
                
                # Check for big fire (marker_id 4)
                if marker_id == 4 and marker_id not in self.big_fires_handled:
                    self.big_fires_handled.add(marker_id)
                    
                    # Spawn two fire response robots
                    fire_x = msg.poses[i].position.x
                    fire_y = msg.poses[i].position.y
                    
                    self.get_logger().info(f'Spawning fire response robots for fire at ({fire_x}, {fire_y})')
                    
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