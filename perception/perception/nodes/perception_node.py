from perception.utils import pipeline
from perception.utils import realsense_loop
from perception.utils import click_points

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

        self.processed_result = realsense_loop.run(
                on_save=pipeline.save_cam,
                on_reset=click_points.reset_points,
                on_click=click_points.mouse_callback,
                update_depth_frame=click_points.update_depth_frame,
                update_color_image=click_points.update_color_image,
                get_points=click_points.click_points.get_saved_points,
        )

    def send_request(self):
        self.req.image = self.processed_result['vis'].tobytes()

        