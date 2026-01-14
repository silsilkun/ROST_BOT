import rclpy
from rclpy.node import Node

from rost_interfaces.srv import estimation_to_control
from rost_interfaces.srv import perception_to_control


class ControlServer(Node):
    def __init__(self):
        super().__init__('control_server')

        # 서비스 생성
        self.estimation_srv = self.create_service(estimation_to_control, 'estimation_to_control', self.on_estimation_request,)
        self.get_logger().info('Control Server ready: /estimation_to_control')
        self.perception_srv = self.create_service(perception_to_control, 'perception_to_control', self.on_perception_request,)
        self.get_logger().info('Control Server ready: /perception_to_control')

    # estimation 한테 데이터 요청
    def on_estimation_request(self, request, response):
        # 리스트 데이터를 받아옴
        values = list(request.values)

        self.get_logger().info(f'Received request: value_count={len(values)}, values={values}')
        response.success = True
        response.message = 'Ok'
        return response

    # perception 한테 데이터 요청
    def on_perception_request(self, request, response):
        # 리스트 받아와서 내부 리스트 인자를 풀어내는 작업
        values = [list(item.data) for item in request.values]
        value_len = [len(item) for item in values]

        self.get_logger().info(f'Received perception data: outer_len={len(values)}, inner_lens={value_len}')
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
