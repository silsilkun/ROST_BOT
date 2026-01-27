from __future__ import annotations

import os
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import List, Optional

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from dotenv import find_dotenv, load_dotenv
from cv_bridge import CvBridge

from estimation.utils.prompt import PromptConfig
from estimation.utils.logic import EstimationLogic
from estimation.utils.utils import ros_image_to_bgr_numpy, bgr_numpy_to_jpeg_bytes
from estimation.utils.estimation_ops import ok_coords, count_from_coords, sanitize_ids, pack_pickup_commands


class EstimationMainNode(Node):
    def __init__(self) -> None:
        super().__init__("estimation_main")
        load_dotenv(find_dotenv(usecwd=True))

        # [수정 포인트] 토픽/파라미터 바뀌면 여기만
        self.declare_parameters(
            "",
            [
                ("coords_topic", "/perception/waste_coordinates"),
                ("image_topic", "/perception/waste_image_raw"),
                ("output_topic", "/estimation/pickup_commands"),
                ("unknown_type_id", -1.0),
                ("max_age_sec", 1.0),
                ("sync_tolerance_sec", 0.25),
                ("drop_if_busy", True),
                ("jpeg_quality", 90),
                ("log_throttle_sec", 2.0),
            ],
        )
        gp = self.get_parameter
        self.coords_topic = gp("coords_topic").value
        self.image_topic = gp("image_topic").value
        self.output_topic = gp("output_topic").value
        self.unknown_type_id = float(gp("unknown_type_id").value)
        self.max_age_sec = float(gp("max_age_sec").value)
        self.sync_tolerance_sec = float(gp("sync_tolerance_sec").value)
        self.drop_if_busy = bool(gp("drop_if_busy").value)
        self.jpeg_quality = int(gp("jpeg_quality").value)
        self.log_throttle_sec = float(gp("log_throttle_sec").value)

        self.logic = EstimationLogic(PromptConfig(), os.getenv("GEMINI_API_KEY"))
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self.create_subscription(Float32MultiArray, self.coords_topic, self._on_coords, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, 10)

        # 세션/상태(꼬임 방지 핵심)
        self._lock = threading.Lock()
        self._busy_lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._sid = 0
        self._seq = 0
        self._state = "IDLE"
        self._coords: Optional[List[float]] = None
        self._stamp = 0.0
        self._last_warn = 0.0
        self._busy = False

        self.get_logger().info(
            f"[Main] Ready. {self.coords_topic} + {self.image_topic} -> {self.output_topic}"
        )

    # ---------- helpers ----------
    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _warn(self, now: float, msg: str) -> None:
        if (now - self._last_warn) >= self.log_throttle_sec:
            self._last_warn = now
            self.get_logger().warn(msg)

    def _reset(self, clear_busy: bool = False) -> None:
        with self._lock:
            self._coords = None
            self._stamp = 0.0
            self._state = "IDLE"
        if clear_busy and self.drop_if_busy:
            with self._busy_lock:
                self._busy = False

    # ---------- inputs ----------
    def _on_coords(self, msg: Float32MultiArray) -> None:
        now = self._now()
        data = list(msg.data)
        ok, reason = ok_coords(data)
        if not ok:
            self._warn(now, f"[Main] invalid coords len={len(data)} -> drop ({reason})")
            self._reset()
            return

        with self._lock:
            # [수정 포인트] req_id/seq 도입 시 sid/seq 갱신 규칙만 교체
            self._sid += 1
            self._seq = 0
            self._state = "READY"
            self._coords = data
            self._stamp = now

    def _on_image(self, msg: Image) -> None:
        now = self._now()
        with self._lock:
            coords = list(self._coords) if self._coords else None
            sid = self._sid
            seq = self._seq + 1
            stamp = self._stamp

        if coords is None:
            self._warn(now, "[Main] image arrived but coords not ready, drop")
            return

        age = now - stamp
        if age > self.max_age_sec:
            self._warn(now, f"[Main] coords stale age={age:.3f}s > {self.max_age_sec:.3f}s, drop")
            self._reset()
            return
        if age > self.sync_tolerance_sec:
            self._warn(
                now,
                f"[Main] coords/image not tight-sync age={age:.3f}s > {self.sync_tolerance_sec:.3f}s, drop",
            )
            self._reset()
            return

        if self.drop_if_busy:
            with self._busy_lock:
                if self._busy:
                    self._warn(now, "[Main] inference busy -> drop new image")
                    return
                self._busy = True

        with self._lock:
            self._state = "BUSY"
            self._seq = seq

        try:
            bgr = ros_image_to_bgr_numpy(self.bridge, msg)
        except Exception as e:
            self.get_logger().error(f"[Main] ros->bgr failed: {e}")
            self._reset(clear_busy=True)
            return

        img = bgr_numpy_to_jpeg_bytes(bgr, jpeg_quality=self.jpeg_quality)
        if img is None:
            self.get_logger().error("[Main] bgr->jpeg failed, drop")
            self._reset(clear_busy=True)
            return

        n = count_from_coords(coords)
        if n <= 0:
            self._warn(now, "[Main] expected_cnt <= 0, drop")
            self._reset(clear_busy=True)
            return

        fut = self._executor.submit(self._infer_and_publish, img, coords, n, sid, seq, stamp)
        fut.add_done_callback(lambda _f: self._clear_busy())

    # ---------- core ----------
    def _infer_and_publish(
        self, img: bytes, coords: List[float], n: int, sid: int, seq: int, stamp: float
    ) -> None:
        with self._lock:
            if sid != self._sid or seq != self._seq:
                return

        if (self._now() - stamp) > self.max_age_sec:
            self._warn(self._now(), "[Main] coords expired during inference, drop")
            self._reset()
            return

        ids = self.logic.run_inference(img, "image/jpeg", n, self.unknown_type_id)
        pickup, reason = pack_pickup_commands(coords, sanitize_ids(ids, self.unknown_type_id))
        if not pickup:
            self._warn(self._now(), f"[Main] failed to pack pickup_commands -> drop ({reason})")
            self._reset()
            return

        with self._lock:
            if sid != self._sid or seq != self._seq:
                return

        self.pub.publish(Float32MultiArray(data=pickup))
        self.get_logger().info(f"[Main] published pickup_commands len={len(pickup)}")
        self._reset()

    def _clear_busy(self) -> None:
        if not self.drop_if_busy:
            return
        with self._busy_lock:
            self._busy = False
