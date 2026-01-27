from __future__ import annotations

import os
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import List, Optional

from rclpy.node import Node
from sensor_msgs.msg import Image
from dotenv import find_dotenv, load_dotenv
from cv_bridge import CvBridge

from estimation.utils.prompt import PromptConfig
from estimation.utils.logic import EstimationLogic
from estimation.utils.utils import ros_image_to_bgr_numpy, bgr_numpy_to_jpeg_bytes
from estimation.utils.estimation_ops import ok_coords, count_from_coords, sanitize_ids, pack_pickup_commands
from rost_interfaces.srv import (
    EstimationToControl,
    PerceptionCoordsToEstimation,
    PerceptionToEstimation,
)


class EstimationMainNode(Node):
    def __init__(self) -> None:
        super().__init__("estimation_main")
        load_dotenv(find_dotenv(usecwd=True))

        # [수정 포인트] 토픽/파라미터 바뀌면 여기만
        self.declare_parameters(
            "",
            [
                ("coords_service", "/perception/waste_coordinates"),
                ("image_service", "/perception/waste_image_raw"),
                ("control_service", "/control/pickup_commands"),
                ("unknown_type_id", -1.0),
                ("max_age_sec", 1.0),
                ("sync_tolerance_sec", 0.25),
                ("drop_if_busy", True),
                ("jpeg_quality", 90),
                ("log_throttle_sec", 2.0),
                ("control_wait_timeout_sec", 2.0),
            ],
        )
        gp = self.get_parameter
        self.coords_service = gp("coords_service").value
        self.image_service = gp("image_service").value
        self.control_service = gp("control_service").value
        self.unknown_type_id = float(gp("unknown_type_id").value)
        self.max_age_sec = float(gp("max_age_sec").value)
        self.sync_tolerance_sec = float(gp("sync_tolerance_sec").value)
        self.drop_if_busy = bool(gp("drop_if_busy").value)
        self.jpeg_quality = int(gp("jpeg_quality").value)
        self.log_throttle_sec = float(gp("log_throttle_sec").value)
        self.control_wait_timeout_sec = float(gp("control_wait_timeout_sec").value)

        self.logic = EstimationLogic(PromptConfig(), os.getenv("GEMINI_API_KEY"))
        self.bridge = CvBridge()

        # Service-based interfaces: perception -> estimation, estimation -> control.
        self._coords_srv = self.create_service(
            PerceptionCoordsToEstimation, self.coords_service, self._handle_coords
        )
        self._image_srv = self.create_service(
            PerceptionToEstimation, self.image_service, self._handle_image
        )
        self._control_client = self.create_client(EstimationToControl, self.control_service)

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
            f"[Main] Ready. services: {self.coords_service} + {self.image_service} -> {self.control_service}"
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

    # ---------- inputs (services) ----------
    def _handle_coords(
        self, request: PerceptionCoordsToEstimation.Request, response: PerceptionCoordsToEstimation.Response
    ) -> PerceptionCoordsToEstimation.Response:
        now = self._now()
        data = list(request.coords)
        ok, reason = ok_coords(data)
        if not ok:
            self._warn(now, f"[Main] invalid coords len={len(data)} -> drop ({reason})")
            self._reset()
            response.success = False
            response.message = f"invalid coords: {reason}"
            return response

        with self._lock:
            # [수정 포인트] req_id/seq 도입 시 sid/seq 갱신 규칙만 교체
            self._sid += 1
            self._seq = 0
            self._state = "READY"
            self._coords = data
            self._stamp = now
        response.success = True
        response.message = "coords accepted"
        return response

    def _handle_image(
        self, request: PerceptionToEstimation.Request, response: PerceptionToEstimation.Response
    ) -> PerceptionToEstimation.Response:
        now = self._now()
        with self._lock:
            coords = list(self._coords) if self._coords else None
            sid = self._sid
            seq = self._seq + 1
            stamp = self._stamp

        if coords is None:
            self._warn(now, "[Main] image arrived but coords not ready, drop")
            response.success = False
            response.message = "coords not ready"
            return response

        age = now - stamp
        if age > self.max_age_sec:
            self._warn(now, f"[Main] coords stale age={age:.3f}s > {self.max_age_sec:.3f}s, drop")
            self._reset()
            response.success = False
            response.message = "coords stale"
            return response
        if age > self.sync_tolerance_sec:
            self._warn(
                now,
                f"[Main] coords/image not tight-sync age={age:.3f}s > {self.sync_tolerance_sec:.3f}s, drop",
            )
            self._reset()
            response.success = False
            response.message = "coords/image sync tolerance exceeded"
            return response

        if self.drop_if_busy:
            with self._busy_lock:
                if self._busy:
                    self._warn(now, "[Main] inference busy -> drop new image")
                    response.success = False
                    response.message = "busy"
                    return response
                self._busy = True

        with self._lock:
            self._state = "BUSY"
            self._seq = seq

        try:
            bgr = ros_image_to_bgr_numpy(self.bridge, request.image)
        except Exception as e:
            self.get_logger().error(f"[Main] ros->bgr failed: {e}")
            self._reset(clear_busy=True)
            response.success = False
            response.message = f"ros->bgr failed: {e}"
            return response

        img = bgr_numpy_to_jpeg_bytes(bgr, jpeg_quality=self.jpeg_quality)
        if img is None:
            self.get_logger().error("[Main] bgr->jpeg failed, drop")
            self._reset(clear_busy=True)
            response.success = False
            response.message = "bgr->jpeg failed"
            return response

        n = count_from_coords(coords)
        if n <= 0:
            self._warn(now, "[Main] expected_cnt <= 0, drop")
            self._reset(clear_busy=True)
            response.success = False
            response.message = "expected count <= 0"
            return response

        fut = self._executor.submit(self._infer_and_send, img, coords, n, sid, seq, stamp)
        try:
            ok, msg = fut.result()
        finally:
            self._clear_busy()

        response.success = bool(ok)
        response.message = str(msg)
        return response

    # ---------- core ----------
    def _infer_and_send(
        self, img: bytes, coords: List[float], n: int, sid: int, seq: int, stamp: float
    ) -> tuple[bool, str]:
        with self._lock:
            if sid != self._sid or seq != self._seq:
                return False, "stale session"

        if (self._now() - stamp) > self.max_age_sec:
            self._warn(self._now(), "[Main] coords expired during inference, drop")
            self._reset()
            return False, "coords expired during inference"

        ids = self.logic.run_inference(img, "image/jpeg", n, self.unknown_type_id)
        pickup, reason = pack_pickup_commands(coords, sanitize_ids(ids, self.unknown_type_id))
        if not pickup:
            self._warn(self._now(), f"[Main] failed to pack pickup_commands -> drop ({reason})")
            self._reset()
            return False, f"failed to pack pickup_commands: {reason}"

        with self._lock:
            if sid != self._sid or seq != self._seq:
                return False, "stale session after inference"

        ok, msg = self._send_to_control(pickup)
        if ok:
            self.get_logger().info(f"[Main] sent pickup_commands len={len(pickup)} to control")
        else:
            self.get_logger().error(f"[Main] failed to send pickup_commands to control: {msg}")
        self._reset()
        return ok, msg

    def _send_to_control(self, pickup: List[float]) -> tuple[bool, str]:
        if not self._control_client.wait_for_service(timeout_sec=self.control_wait_timeout_sec):
            return False, f"control service not available: {self.control_service}"

        req = EstimationToControl.Request()
        req.values = [float(x) for x in pickup]
        fut = self._control_client.call_async(req)
        try:
            import rclpy

            rclpy.spin_until_future_complete(self, fut, timeout_sec=self.control_wait_timeout_sec)
        except Exception as e:  # spin_until_future_complete 방어
            return False, f"spin_until_future_complete failed: {e}"

        if not fut.done():
            return False, "control service call timed out"
        if fut.exception() is not None:
            return False, f"control service call failed: {fut.exception()}"

        res = fut.result()
        return bool(res.success), str(res.message or "")

    def _clear_busy(self) -> None:
        if not self.drop_if_busy:
            return
        with self._busy_lock:
            self._busy = False
