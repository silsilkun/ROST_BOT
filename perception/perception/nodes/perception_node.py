import rclpy
from rclpy.node import Node
from perception.srv import to_estimation

import sys
import termios
import tty

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class PerceptionClient(Node):
    def __init__(self):
        super().__init__('perception_client')
        self.client_to_estimation = self.create_client(to_estimation, 'send_boxed_image')
        self.client_to_control = self.create_client(to_control, 'send_coordinates')

        while not self.client_to_estimation.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        while not self.client_to_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('컨트롤 서비스 대기 중...')

        self.get_logger().info('스페이스바를 누르면 이미지 전송')

    def send_request_to_estimation(self):
        request = to_estimation.Request()
        request.image = None  # Placeholder for actual image data

        is_success = self.client_to_estimation.call_async(request)
        is_success.add_done_callback(self.response_callback)

    def response_callback(self, is_successful):
        response = is_successful.result()
        self.get_logger().info(f"추론 서버 응답: {response.success}")


def main():
    rclpy.init()
    node = PerceptionClient()

    try:
        while rclpy.ok():
            key = get_key()

            if key == ' ':
                node.get_logger().info('spacebar 입력 → 요청 전송')
                node.send_request_to_estimation()

            elif key == 'q':
                break

            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()