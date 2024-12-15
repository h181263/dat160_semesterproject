import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Int64
from interfaces.srv import SetMarkerPosition

class RobotHandlerClass(Node):
    def __init__(self, robot_namespace):
        super().__init__('RobotHandlerNode', namespace=robot_namespace)

        self.sub_lidar = self.create_subscription(LaserScan, 'scan', self.clbk_lidar, 10)
        
        self.pub_marker_id = self.create_publisher(Int64, 'marker_id', 10)

        self.lidar_value = 100.0

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.marker_id = -1
        self.big_fire_detected = False

        self.srv = self.create_service(SetMarkerPosition, 'set_marker_position', self.set_marker_position_callback)

        self.client_set_marker_pos = self.create_client(SetMarkerPosition, 'set_marker_position')
        while not self.client_set_marker_pos.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again..')
        self.get_logger().info('Connected to set_marker_position service.')

    def clbk_lidar(self, msg):
        self.lidar_value = msg.ranges[180]

        if self.lidar_value < 2.0:
            marker_id_msg = Int64()
            marker_id_msg.data = 4
            self.pub_marker_id.publish(marker_id_msg)

    def timer_callback(self):
        if self.big_fire_detected:
            if self.are_two_robots_within_distance():
                self.pub_marker_id.publish(self.marker_id)

    def are_two_robots_within_distance(self):
        return False # no solution yet
    
    def set_marker_position_callback(self, request, response):
        self.get_logger().info(f'Recieved set_marker_position request: marker_id = {request.marker_id}')

        response.success = True
        return response
    
    def report_marker_position(self, marker_id, marker_position):
        request = SetMarkerPosition.Request()
        request.marker_id = marker_id
        request.marker_position = marker_position

        future = self.client_set_marker_pos.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().accepted:
                self.get_logger().info(f"Marker {marker_id} position reported successfully.")
            else:
                self.get_logger().warn(f"Failed to report Marker {marker_id} position")
        else:
            self.get_logger().error("Service call failed. No response.")

def main(args=None):
    rclpy.init(args=args)

    robot_namespace_0 = '/tb3_0'
    robot_handler_0 = RobotHandlerClass(robot_namespace_0)

    robot_namespace_1 = '/tb3_1'
    robot_handler_1 = RobotHandlerClass(robot_namespace_1)

    rclpy.spin(robot_handler_0)
    rclpy.spin(robot_handler_1)

    robot_handler_0.destroy_node()
    robot_handler_1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()