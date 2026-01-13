from rost_interfaces.srv import perception_to_estimation

import rclpy
from rclpy.node import Node

class PerceptionNode(Node):


    def __init__(self):
        super().__init__('perception_node')
        self.srv = self.create_client(perception_to_estimation, 'perception_to_estimation')
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = perception_to_estimation.Request()