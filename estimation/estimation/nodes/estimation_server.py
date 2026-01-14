import rclpy
from rclpy.node import Node

from rost_interfaces.srv import perception_to_estimation




class EstimationServer(Node):
    def __init__(self):
        super().__init__('estimation_server')
        
        #서비스 생성
        self.srv = self.create_service(perception_to_estimation, 'perception_to_estimation', self.on_request)
        self.get_logger().info("Estimation Server ready: /perception_to_estimation")
        
        
    def on_request(self, request, response):
        img_len = len(request.image)
        
        self.get_logger().info(f"Received request: image bytes = {img_len}")
        
        response.success = True
        response.message = f"Ok (received {img_len} bytes)"
        return response
    
    
def main(args=None):
    rclpy.init()
    node = EstimationServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()