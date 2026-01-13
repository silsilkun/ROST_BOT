from rost_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.srv = self.create_client(AddTwoInts, 'add_two_ints')

        self.get_logger().info('Perception Node has been started and service is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Adding {request.a} + {request.b} = {response.sum}')
        return response
    
    def call_add_two_ints(self, a, b):
        while not self.srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Service call result: {future.result().sum}')
            return future.result().sum
        else:
            self.get_logger().error('Service call failed')
            return None