



from rost_interfaces.srv import EstimationToControl

import rclpy
from rclpy.node import Node




class EstimationNode(Node):

    def __init__(self):
        super().__init__('estimation_node')
        self.srv = self.create_client(EstimationToControl, 'estimation_to_control')
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = EstimationToControl.Request()
        
        
