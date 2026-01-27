import time
import threading
from cv_bridge import CvBridge
import numpy as np

from perception.utils import pipeline
from perception.utils import realsense_loop
from perception.utils import click_points

from rost_interfaces.srv import PerceptionToEstimation
from rost_interfaces.action import Circulation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


last_result = None
running = True

class PerceptionNode(Node):

    def __init__(self):

        self.bridge = CvBridge()
        super().__init__('perception_node')

        def Camera_Process(self):
            pass
        ''' Create an action client for the Circulation action '''
        self.action_client = ActionClient(self, Circulation, 'circulation_action')
        ''' Create a client for the PerceptionToEstimation service '''
        self.srv_client = self.create_client(PerceptionToEstimation, 'perception_to_estimation')

    # Create action client definitions
    def send_goal(self):
        goal = Circulation.Goal()
        goal.start_signal = "동작 시작"

        self.action_client.wait_for_server()
        self.send_action_goal = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self.send_action_goal.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg}")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result}")

        # Circulating function
        data_to_send_request()
    ''' End of action client definitions '''


    # Create service client definitions
    def send_request(self, input_data):
        req = PerceptionToEstimation.Request()
        # OpenCV → ROS Image
        req.image = self.bridge.cv2_to_imgmsg(
            input_data['color'],
            encoding='bgr8'
        )

        req.vis = self.bridge.cv2_to_imgmsg(
            input_data['vis'],
            encoding='bgr8'
        )
        
        req.tcoordinates = [
            float(v)
            for arr in input_data['boxes']
            for v in np.array(arr).flatten()
        ]
        req.ccoordinates = [float(v) for point in input_data['clicked_world_xy_list'] for v in point]

        self.srv_client.wait_for_service()
        self.future = self.srv_client.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service response received: {response.success_message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    ''' End of service client definitions '''




def main(args=None):
    rclpy.init(args=args)

    perception_node = PerceptionNode()

    # ROS spin은 별도 스레드
    threading.Thread(
        target=rclpy.spin,
        args=(perception_node,),
        daemon=True
    ).start()

    def on_save_and_store():
        result = pipeline.save_cam()
        if result is None:
            return None

        perception_node.send_request(result)
        return result

    realsense_loop.run(
        on_save=on_save_and_store,
        on_reset=click_points.reset_points,
        on_click=click_points.mouse_callback,
        update_depth_frame=click_points.update_depth_frame,
        update_color_image=click_points.update_color_image,
        get_points=click_points.get_saved_points,
    )

    perception_node.destroy_node()
    rclpy.shutdown()
