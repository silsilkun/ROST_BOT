import rclpy
from rclpy.node import Node

from rost_interfaces.srv import estimation_to_control
from rost_interfaces.srv import perception_to_control


class ControlServer(Node):
    def __init__(self):
        super().__init__('control_server')

        # 서비스 생성
        self.estimation_srv = self.create_service(estimation_to_control, 'estimation_to_control', self.on_request,)
        self.get_logger().info('Control Server ready: /estimation_to_control')
        self.perception_srv = self.create_service(perception_to_control, 'perception_to_control', self.on_perception_request,)
        self.get_logger().info('Control Server ready: /perception_to_control')

    # estimation 한테 데이터 요청
    def on_estimation_request(self, request, response):
        values = list(request.values)
        # 리스트의 인자가 5개가 아니면 오류 처리
        if len(values) != 5:
            self.get_logger().info(f'Received request: expected 5 values, got {len(values)}')
            response.success = False
            response.message = f'Expected 5 values, got {len(values)}'
            return response

        self.get_logger().info(f'Received request: values={values}')
        response.success = True
        response.message = 'Ok'
        return response

    # perception 한테 데이터 요청
    def on_perception_request(self, request, response):
        nested_values = [list(item.data) for item in request.values]
        inner_lengths = [len(item) for item in nested_values]

        self.get_logger().info(f'Received perception data: outer_len={len(nested_values)}, inner_lens={inner_lengths}')
        response.success = True
        response.message = 'Ok'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControlServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
