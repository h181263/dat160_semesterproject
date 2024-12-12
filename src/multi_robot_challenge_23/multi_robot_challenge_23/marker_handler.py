from geometry_msgs.msg import Point, Pose
from scoring_interfaces.srv import SetMarkerPosition
import rclpy

class MarkerHandler:
    def __init__(self, node):
        self.node = node
        self.reported_markers = set()
        
        # Create scoring service client
        self.score_client = node.create_client(
            SetMarkerPosition,
            'report_marker'
        )
        
        # Wait for scoring service
        while not self.score_client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Waiting for scoring service...')

    def handle_marker(self, robot_id, marker_id, marker_pose):
        if marker_id in self.reported_markers:
            return
            
        # Convert marker pose to world coordinates
        world_position = self.transform_to_world(marker_pose)
        
        # Handle different marker types
        if marker_id == 4:  # Big fire
            self.node.coordinator.coordinate_big_fire_response(world_position)
            
        # Report marker to scoring system
        self.report_marker(marker_id, world_position)

    async def report_marker(self, marker_id, position):
        request = SetMarkerPosition.Request()
        request.marker_id = marker_id
        request.marker_position = position
        
        try:
            response = await self.score_client.call_async(request)
            if response.accepted:
                self.reported_markers.add(marker_id)
                self.node.get_logger().info(f'Successfully reported marker {marker_id}')
            else:
                self.node.get_logger().warn(f'Failed to report marker {marker_id}')
        except Exception as e:
            self.node.get_logger().error(f'Error reporting marker: {str(e)}')

    def transform_to_world(self, marker_pose):
        # Implement coordinate transformation from camera frame to world frame
        return Point(
            x=marker_pose.position.x,
            y=marker_pose.position.y,
            z=marker_pose.position.z
        ) 