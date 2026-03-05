from rost_interfaces.srv import EstimationToControl

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from control.utils.recycle_260220 import RecycleNew
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DEFAULT_PICK_Z_CM = 160.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class ControlNode(Node):
    def __init__(self, robot: RecycleNew):
        super().__init__('control_node')
        self._robot = robot
        self._srv = self.create_service(
            EstimationToControl,
            'estimation_to_control',
            self.handle_estimation_to_control,
        )
        self.get_logger().info('Control Node ready: /estimation_to_control')

    def handle_estimation_to_control(self, request, response):
        data = list(request.data or [])
        if len(data) < 7:
            response.success = False
            response.message = f"Expected 7 floats, got {len(data)}"
            return response

        # input format from estimation (all in mm except angle in deg):
        # [type_id, tx_mm, ty_mm, short_side_mm, angle_deg, bin_x_mm, bin_y_mm]
        # recycle_260220 expects:
        # [type_id, tx_mm, ty_mm, edge_mm, angle_deg, bin_x_mm, bin_y_mm]
        type_id, tx, ty, short_side_mm, angle, bx, by = data[:7]
        edge_mm = float(short_side_mm)
        self.get_logger().info(
            f"Received: type={type_id}, tx={tx:.2f}, ty={ty:.2f}, "
            f"short_side_mm={short_side_mm:.1f}, angle={angle:.1f}, edge_mm={edge_mm:.1f}"
        )
        trash_list = [float(type_id), float(tx), float(ty), edge_mm, float(angle), float(bx), float(by)]

        try:
            self._robot.run(trash_list)
        except Exception as exc:
            response.success = False
            response.message = f"Control run failed: {exc}"
            return response

        response.success = True
        response.message = "OK"
        return response


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    robot = RecycleNew()
    control_node = ControlNode(robot)

    executor = MultiThreadedExecutor()
    executor.add_node(dsr_node)
    executor.add_node(robot)
    executor.add_node(control_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        robot.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
