# judge_node.py (FINAL FLOW: Image(bgr8) + Coordinates(N*5) -> Type IDs(N))
import os
import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from dotenv import find_dotenv, load_dotenv
from cv_bridge import CvBridge

from estimation.utils.prompt import PromptConfig
from estimation.utils.utils import validate_waste_coordinates_flat, ros_image_to_bgr_numpy, bgr_numpy_to_jpeg_bytes
from estimation.utils.logic import EstimationLogic


class EstimationJudgeNode(Node):
    """
    Input:
      - coords_topic: Float32MultiArray, [tmp_id,x,y,z,angle]*N
      - image_topic : sensor_msgs/Image, encoding=bgr8 (numpy BGR uint8)
    Output:
      - output_topic: Float32MultiArray, [type_id]*N
    """

    def __init__(self):
        super().__init__("estimation_judge")
        load_dotenv(find_dotenv(usecwd=True))

        self.declare_parameters(
            namespace="",
            parameters=[
                ("coords_topic", "/perception/waste_coordinates"),
                ("image_topic", "/perception/waste_image_raw"),
                ("output_topic", "/estimation/type_id"),
                ("unknown_type_id", -1.0),
                ("max_age_sec", 1.0),
                ("drop_if_busy", True),
                ("jpeg_quality", 90),
            ],
        )

        self.coords_topic = self.get_parameter("coords_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.unknown_type_id = float(self.get_parameter("unknown_type_id").value)
        self.max_age_sec = float(self.get_parameter("max_age_sec").value)
        self.drop_if_busy = bool(self.get_parameter("drop_if_busy").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # Logic (Gemini)
        cfg = PromptConfig()
        self.get_logger().info(
            f"[Judge][DBG] model={cfg.default_model} max_tokens={cfg.default_max_tokens} temp={cfg.default_temp}"
        )
        self.logic = EstimationLogic(cfg, os.getenv("GEMINI_API_KEY"))
        self.bridge = CvBridge()

        # Publishers/Subscribers
        self.pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.create_subscription(Float32MultiArray, self.coords_topic, self._on_coords, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, 10)

        # Cache
        self._last_coords = None               # List[float], len=5N
        self._last_coords_time = None          # rclpy.time.Time

        # Async inference (Control/Estimator executor block 방지)
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._busy_lock = threading.Lock()
        self._busy = False

        self.get_logger().info(
            f"[Judge] Ready. coords={self.coords_topic} + image={self.image_topic}(bgr8) -> {self.output_topic}"
        )

    # ---------- Input 1: Coordinates ----------
    def _on_coords(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)

        # 무결성(길이, NaN/Inf)
        if not validate_waste_coordinates_flat(data):
            self.get_logger().warn(f"[Judge] invalid coords len={len(data)} (need N*5), drop")
            return

        self._last_coords = data
        self._last_coords_time = self.get_clock().now()

    # ---------- Input 2: Image(bgr8) ----------
    def _on_image(self, msg: Image) -> None:
        # coords 준비 여부
        if self._last_coords is None or self._last_coords_time is None:
            self.get_logger().warn("[Judge] image arrived but coords not ready, drop")
            return

        # coords freshness
        now = self.get_clock().now()
        coords_age = (now - self._last_coords_time).nanoseconds / 1e9
        if coords_age > self.max_age_sec:
            self.get_logger().warn(
                f"[Judge] coords stale age={coords_age:.3f}s > {self.max_age_sec:.3f}s, drop"
            )
            return

        # busy-drop (queue 무한증가 방지)
        if self.drop_if_busy:
            with self._busy_lock:
                if self._busy:
                    self.get_logger().warn("[Judge] inference busy -> drop new image (skip)")
                    return
                self._busy = True

        # ROS Image(bgr8) -> numpy(BGR uint8)
        try:
            bgr = ros_image_to_bgr_numpy(self.bridge, msg)
        except Exception as e:
            self.get_logger().error(f"[Judge] ros->bgr failed: {e}")
            self._clear_busy()
            return

        # numpy(BGR) -> JPEG bytes
        img_bytes = bgr_numpy_to_jpeg_bytes(bgr, jpeg_quality=self.jpeg_quality)
        if img_bytes is None:
            self.get_logger().error("[Judge] bgr->jpeg failed, drop")
            self._clear_busy()
            return

        # N은 coords 기반으로 결정 (expected_count 제거)
        expected_cnt = len(self._last_coords) // 5

        # Async inference
        fut = self._executor.submit(self._infer_and_publish, img_bytes, expected_cnt)
        fut.add_done_callback(lambda _f: self._clear_busy())

    # ---------- Core ----------
    def _infer_and_publish(self, img_bytes: bytes, expected_cnt: int) -> None:
        ids = self.logic.run_inference(
            img_bytes,
            "image/jpeg",
            expected_cnt,
            self.unknown_type_id
        )
        self.pub.publish(Float32MultiArray(data=[float(x) for x in ids]))
        self.get_logger().info(f"[Judge] published type_ids N={len(ids)}")

    def _clear_busy(self) -> None:
        if not self.drop_if_busy:
            return
        with self._busy_lock:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = EstimationJudgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # [추가] 비동기 추론 스레드 정리 (Control loop block 방지 + clean shutdown)
        node._executor.shutdown(wait=False)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()