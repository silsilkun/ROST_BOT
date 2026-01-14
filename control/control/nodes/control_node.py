import rclpy
from rclpy.node import Node

from rost_interfaces.srv import perception_to_estimation


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.srv = self.create_service(estimation_to_control, 'estimation_to_control', self.handle_perception_request,)
        self.get_logger().info('perception_to_estimation service ready')

    def handle_perception_request(self, request, response):
        image_msg = request.image
        data_len = len(image_msg.data) if image_msg.data else 0
        self.get_logger().info(f'received image data bytes: {data_len}')

        response.success = True
        response.message = 'image received'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
