from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rost_interfaces.srv import (
    EstimationToControl,
    PerceptionCoordsToEstimation,
    PerceptionToEstimation,
)


class EstimationStandaloneTest(Node):
    """Perception/Control 없이 service 흐름을 단독 테스트하는 간단한 클라이언트."""

    def __init__(self) -> None:
        super().__init__("estimation_standalone_test")

        # [수정 포인트] 서비스명/데이터를 바꾸고 싶으면 여기만
        self.declare_parameters("", [
            ("coords_service", "/perception/waste_coordinates"),
            ("image_service", "/perception/waste_image_raw"),
            ("control_service", "/control/pickup_commands"),
            ("period_sec", 1.5),
            # [tmp_id,x,y,z,angle]*N  (여기서는 N=2)
            ("coords_data", [0.0, 0.1, 0.2, 0.3, 0.0, 1.0, 0.4, 0.5, 0.6, 0.1]),
            ("image_width", 640),
            ("image_height", 480),
        ])
        gp = self.get_parameter
        self.coords_service = gp("coords_service").value
        self.image_service = gp("image_service").value
        self.control_service = gp("control_service").value
        self.period_sec = float(gp("period_sec").value)
        self.coords_data = [float(x) for x in gp("coords_data").value]
        self.w = int(gp("image_width").value)
        self.h = int(gp("image_height").value)

        self.bridge = CvBridge()
        self.coords_client = self.create_client(PerceptionCoordsToEstimation, self.coords_service)
        self.image_client = self.create_client(PerceptionToEstimation, self.image_service)

        # Control service mock: estimation -> control 경로를 관찰하기 위함
        self.control_server = self.create_service(
            EstimationToControl, self.control_service, self._on_control_request
        )

        self._timer = self.create_timer(self.period_sec, self._tick)
        self.get_logger().info(
            f"[StandaloneTest] calling {self.coords_service} + {self.image_service}, serving {self.control_service}"
        )

    def _tick(self) -> None:
        # coords 먼저, 바로 이어서 image를 호출해 sync tolerance를 통과시킴
        if not self.coords_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"[StandaloneTest] coords service not ready: {self.coords_service}")
            return
        if not self.image_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f"[StandaloneTest] image service not ready: {self.image_service}")
            return

        coords_req = PerceptionCoordsToEstimation.Request()
        coords_req.coords = [float(x) for x in self.coords_data]
        coords_ok, coords_msg = self._call_service(self.coords_client, coords_req, "coords")
        if not coords_ok:
            self.get_logger().warn(f"[StandaloneTest] coords call failed: {coords_msg}")
            return

        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)  # bgr8
        image_req = PerceptionToEstimation.Request()
        image_req.image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        image_ok, image_msg = self._call_service(self.image_client, image_req, "image")
        self.get_logger().info(
            f"[StandaloneTest] coords+image called -> image_ok={image_ok} msg={image_msg}"
        )

    def _call_service(self, client, request, label: str) -> tuple[bool, str]:
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done():
            return False, f"{label} call timed out"
        if fut.exception() is not None:
            return False, f"{label} call raised: {fut.exception()}"
        res = fut.result()
        return bool(res.success), str(res.message or "")

    def _on_control_request(
        self, request: EstimationToControl.Request, response: EstimationToControl.Response
    ) -> EstimationToControl.Response:
        data = list(request.values)
        n = len(data) // 5
        self.get_logger().info(
            f"[StandaloneTest] control request N={n} len={len(data)} sample={data[:10]}"
        )
        response.success = True
        response.message = "mock control accepted"
        return response


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
