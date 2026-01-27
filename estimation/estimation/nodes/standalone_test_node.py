from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class EstimationStandaloneTest(Node):
    """Perception/Control 없이 main 노드를 단독 테스트하는 간단한 발행기."""

    def __init__(self) -> None:
        super().__init__("estimation_standalone_test")

        # [수정 포인트] 토픽/데이터를 바꾸고 싶으면 여기만
        self.declare_parameters("", [
            ("coords_topic", "/perception/waste_coordinates"),
            ("image_topic", "/perception/waste_image_raw"),
            ("output_topic", "/estimation/pickup_commands"),
            ("period_sec", 1.5),
            # [tmp_id,x,y,z,angle]*N  (여기서는 N=2)
            ("coords_data", [0.0, 0.1, 0.2, 0.3, 0.0, 1.0, 0.4, 0.5, 0.6, 0.1]),
            ("image_width", 640),
            ("image_height", 480),
        ])
        gp = self.get_parameter
        self.coords_topic = gp("coords_topic").value
        self.image_topic = gp("image_topic").value
        self.output_topic = gp("output_topic").value
        self.period_sec = float(gp("period_sec").value)
        self.coords_data = [float(x) for x in gp("coords_data").value]
        self.w = int(gp("image_width").value)
        self.h = int(gp("image_height").value)

        self.bridge = CvBridge()
        self.pub_coords = self.create_publisher(Float32MultiArray, self.coords_topic, 10)
        self.pub_image = self.create_publisher(Image, self.image_topic, 10)
        self.create_subscription(Float32MultiArray, self.output_topic, self._on_output, 10)

        self._timer = self.create_timer(self.period_sec, self._tick)
        self.get_logger().info(
            f"[StandaloneTest] publishing to {self.coords_topic} + {self.image_topic}, listening on {self.output_topic}"
        )

    def _tick(self) -> None:
        # coords 먼저, 바로 이어서 image를 발행해 sync tolerance를 통과시킴
        self.pub_coords.publish(Float32MultiArray(data=self.coords_data))
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)  # bgr8
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        self.get_logger().info("[StandaloneTest] published coords+image")

    def _on_output(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        n = len(data) // 5
        self.get_logger().info(f"[StandaloneTest] got pickup_commands N={n} len={len(data)} data={data[:10]}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimationStandaloneTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
