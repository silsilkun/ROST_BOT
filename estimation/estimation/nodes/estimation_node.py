



from rost_interfaces.srv import estimation_to_control

import rclpy
from rclpy.node import Node




class EstimationNode(Node):

    def __init__(self):
        super().__init__('estimation_node')
        self.srv = self.create_client(estimation_to_control, 'estimation_to_control')
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = estimation_to_control.Request()
        
        