import rclpy
from rclpy.node import Node

from rost_interfaces.msg import Float64Array
from rost_interfaces.srv import PerEstToControl


class PerEstServer(Node):
    def __init__(self):
        super().__init__('per_est_server')
        self.srv = self.create_service(PerEstToControl, 'per_est_to_control', self.on_request)
        self.get_logger().info('Perception+Estimation Server ready: /per_est_to_control')

    def on_request(self, request, response):
        # TODO: Replace with real perception + estimation pipeline.
        trash = Float64Array()
        trash.data = [1.0, 400.0, 200.0, 130.0, 60.0]

        bin_loc = Float64Array()
        bin_loc.data = [300.0, 430.0]

        response.success = True
        response.message = 'Ok (dummy data)'
        response.trash_list = [trash]
        response.bin_list = [bin_loc]
        return response
