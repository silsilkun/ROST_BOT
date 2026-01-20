import rclpy

from control.nodes.control_client import ControlClient


def main(args=None):
    rclpy.init(args=args)
    node = ControlClient()
    try:
        trash_list, bin_list = node.request_data()
        node.get_logger().info(f"trash_list={trash_list}")
        node.get_logger().info(f"bin_list={bin_list}")
    finally:
        node.destroy_node()
        rclpy.shutdown()