from rost_interfaces.srv import PerceptionToEstimation
from rost_interfaces.srv import EstimationToControl

import rclpy
from rclpy.node import Node




class EstimationNode(Node):

    def __init__(self):
        super().__init__('estimation_node')
        
        # Create a server for the PerceptionToEstimation service
        self.srv_server = self.create_service(PerceptionToEstimation, 'perception_to_estimation', self.handle_perception_to_estimation_request)
        # Create service server definitions
        def handle_perception_to_estimation_request(self, request, response):
            raw_image = request.image
            vis_image = request.vis
            Tcoordinates = request.Tcoordinates
            Ccoordinates = request.Ccoordinates

            self.get_logger().info('Received request from Perception Node')

            # Dummy processing (replace with actual estimation logic)
            success_message = "이미지 및 좌표 수신 완료"

            response.success_message = success_message
            return response
        # End of service server definitions

        # Create a client for the EstimationToControl service
        self.srv_client = self.create_client(EstimationToControl, 'estimation_to_control')
        # Create service client definitions
        def send_request(self, estimation_data):
            req = EstimationToControl.Request()
            req.estimation_data = estimation_data

            self.srv_client.wait_for_service()
            self.future = self.srv_client.call_async(req)
            self.future.add_done_callback(self.service_response_callback)

        def service_response_callback(self, future):
            try:
                response = future.result()
                self.get_logger().info(f"Service response received: {response.success_message}")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
        # End of service client definitions


def main(args=None):
    rclpy.init(args=args)

    estimation_node = EstimationNode()

    rclpy.spin(estimation_node)

    estimation_node.handle_perception_to_estimation_request()
    estimation_node.send_request()

    estimation_node.destroy_node()
    rclpy.shutdown()