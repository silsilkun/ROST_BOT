from rost_interfaces.srv import EstimationToControl
from rost_interfaces.action import Circulation

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from control.control.utils import recycle_new
from control.control.utils.recycle_new import RecycleNew
import DR_init

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Create an action server for the Circulation action
        self.action_server = ActionServer(self, Circulation, 'circulation_action', self.execute_callback)
        # Create a service server for EstimationToControl service
        self.srv = self.create_service(EstimationToControl, 'estimation_to_control', self.handle_estimation_to_control)

    ''' Create action server definitions '''
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
    ''' End of action server definitions '''
    
    ''' Create service server definitions '''
    def handle_estimation_to_control(self, request, response):
        self.get_logger().info('Received request from Estimation Node')

        self.trash_coordinates = request.tcoordinates
        self.can_coordinates = request.ccoordinates

        # Dummy processing (replace with actual control logic)
        response.control_signal = "좌표 수신 완료"

        return response
    
def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=recycle_new.ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    control_node = ControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    executor.spin()
    trash = control_node.trash_coordinates
    bin_pos = control_node.can_coordinates

    robot = RecycleNew()
    robot.run(trash, bin_pos)

    control_node.destroy_node()
    robot.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()