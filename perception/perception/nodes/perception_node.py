from perception.utils import pipeline
from perception.utils import realsense_loop
from perception.utils import click_points

from rost_interfaces.srv import PerceptionToEstimation
from rost_interfaces.action import Circulation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient




class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        def Camera_Process(self):
            pass

        # Create an action client for the Circulation action
        self.action_client = ActionClient(self, Circulation, 'circulation_action')
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
        # End of action client definitions


        # Create a client for the PerceptionToEstimation service
        self.srv_client = self.create_client(PerceptionToEstimation, 'perception_to_estimation')
        # Create service client definitions
        def send_request(self, input_data):
            req = PerceptionToEstimation.Request()
            req.image = input_data.image
            req.vis = input_data.vis
            req.Tcoordinates = input_data.Tcoordinates
            req.Ccoordinates = input_data.Ccoordinates

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

    perception_node = PerceptionNode()
    perception_node.send_goal()
    perception_node.send_request()

    rclpy.spin(perception_node)

    perception_node.destroy_node()
    rclpy.shutdown()