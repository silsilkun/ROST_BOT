# judge_core.py
import threading
from concurrent.futures import ThreadPoolExecutor

from std_msgs.msg import Float32MultiArray

from estimation.utils.logic import EstimationLogic
from estimation.utils.prompt import PromptConfig
from estimation.utils.utils import (
    validate_waste_coordinates_flat,
    ros_image_to_bgr_numpy,
    bgr_numpy_to_jpeg_bytes,
)


class JudgeCore:
    def __init__(self, node, pub, unknown_type_id, max_age_sec, drop_if_busy, jpeg_quality):
        self.node = node
        self.pub = pub
        self.unknown_type_id = float(unknown_type_id)
        self.max_age_sec = float(max_age_sec)
        self.drop_if_busy = bool(drop_if_busy)
        self.jpeg_quality = int(jpeg_quality)

        cfg = PromptConfig()
        self.logic = EstimationLogic(cfg, self._get_api_key())
        self.bridge = self._make_bridge()

        self._last_coords = None
        self._last_coords_time = None
        self._last_raw_bgr = None
        self._last_vis_bgr = None

        self._executor = ThreadPoolExecutor(max_workers=1)
        self._busy_lock = threading.Lock()
        self._busy = False

    def shutdown(self):
        self._executor.shutdown(wait=False)

    def on_coords(self, msg):
        data = list(msg.data)
        if not validate_waste_coordinates_flat(data):
            self.node.get_logger().warn(
                f"[Judge] invalid coords len={len(data)} (need N*5), drop"
            )
            return
        self._last_coords = data
        self._last_coords_time = self.node.get_clock().now()

    def on_image_raw(self, msg):
        self._last_raw_bgr = self._to_bgr(msg, "raw")
        if self._last_raw_bgr is not None:
            self._maybe_infer()

    def on_image_vis(self, msg):
        self._last_vis_bgr = self._to_bgr(msg, "vis")
        if self._last_vis_bgr is not None:
            self._maybe_infer()

    def _maybe_infer(self):
        if self._last_coords is None or self._last_coords_time is None:
            self.node.get_logger().warn("[Judge] image arrived but coords not ready, drop")
            return
        now = self.node.get_clock().now()
        coords_age = (now - self._last_coords_time).nanoseconds / 1e9
        if coords_age > self.max_age_sec:
            self.node.get_logger().warn(
                f"[Judge] coords stale age={coords_age:.3f}s > {self.max_age_sec:.3f}s, drop"
            )
            return
        if self._last_raw_bgr is None or self._last_vis_bgr is None:
            return
        if self.drop_if_busy:
            with self._busy_lock:
                if self._busy:
                    self.node.get_logger().warn("[Judge] inference busy -> drop")
                    return
                self._busy = True

        raw_bytes = bgr_numpy_to_jpeg_bytes(self._last_raw_bgr, jpeg_quality=self.jpeg_quality)
        vis_bytes = bgr_numpy_to_jpeg_bytes(self._last_vis_bgr, jpeg_quality=self.jpeg_quality)
        if raw_bytes is None or vis_bytes is None:
            self.node.get_logger().error("[Judge] bgr->jpeg failed, drop")
            self._clear_busy()
            return

        expected_cnt = len(self._last_coords) // 5
        images = [(raw_bytes, "image/jpeg"), (vis_bytes, "image/jpeg")]
        fut = self._executor.submit(self._infer_and_publish, images, expected_cnt)
        fut.add_done_callback(lambda _f: self._clear_busy())

    def _infer_and_publish(self, images, expected_cnt):
        ids = self.logic.run_inference(images, expected_cnt, self.unknown_type_id)
        self.pub.publish(Float32MultiArray(data=[float(x) for x in ids]))
        self.node.get_logger().info(f"[Judge] published type_ids N={len(ids)}")

    def _clear_busy(self):
        if not self.drop_if_busy:
            return
        with self._busy_lock:
            self._busy = False

    def _to_bgr(self, msg, tag):
        try:
            return ros_image_to_bgr_numpy(self.bridge, msg)
        except Exception as e:
            self.node.get_logger().error(f"[Judge] {tag} ros->bgr failed: {e}")
            return None

    def _make_bridge(self):
        try:
            from cv_bridge import CvBridge
            return CvBridge()
        except Exception as e:
            self.node.get_logger().error(f"[Judge] CvBridge not available: {e}")
            return None

    def _get_api_key(self):
        import os
        try:
            from dotenv import find_dotenv, load_dotenv
            load_dotenv(find_dotenv(usecwd=True))
        except Exception:
            pass
        return os.getenv("GEMINI_API_KEY")
