from rost_interfaces.srv import EstimationToControl
from rost_interfaces.action import Circulation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Create an action server for the Circulation action
        self.action_server = ActionServer(self, Circulation, 'circulation_action', self.execute_callback)
        # Create action server definitions
        async def execute_callback(self, goal_handle):
            self.get_logger().info('Received goal request from Estimation Node')

            feedback = goal_handle.Feedback()
            feedback.status_signal = "동작 진행 중"
            goal_handle.publish_feedback(feedback)

            # Dummy processing (replace with actual control logic)
            result = Circulation.Result()
            result.completion_signal = "동작 완료"

            goal_handle.succeed()

            return result
        
        def timer_callback(self):
            if self.processing_complete:
                self.timer.cancel()
        # End of action server definitions