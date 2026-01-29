# core.py
from __future__ import annotations

import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

from rost_interfaces.srv import (
    PerceptionToEstimationRawImage,
    PerceptionToEstimationVisImage,
    PerceptionToEstimationTrashPoints,
    PerceptionToEstimationBinPoints,
    EstimationToControlTrashBinPoints,
)
from rost_interfaces.msg import TrashPoint, BinPoint, TrashBinPoint

from estimation.utils.prompt import PromptConfig
from estimation.utils.logic import EstimationLogic
from estimation.utils.utils import ros_image_to_bgr_numpy, bgr_numpy_to_jpeg_bytes


@dataclass
class _InputSet:
    raw: Optional[PerceptionToEstimationRawImage.Request] = None
    vis: Optional[PerceptionToEstimationVisImage.Request] = None
    trash: Optional[PerceptionToEstimationTrashPoints.Request] = None
    binp: Optional[PerceptionToEstimationBinPoints.Request] = None


class EstimationServiceApp(Node):
    def __init__(self):
        super().__init__("estimation_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("max_age_sec", 1.0),
                ("drop_if_busy", True),
                ("jpeg_quality", 90),
                ("unknown_type_id", -1.0),
                ("dry_run", False),
            ],
        )
        self.max_age_sec = float(self.get_parameter("max_age_sec").value)
        self.drop_if_busy = bool(self.get_parameter("drop_if_busy").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.unknown_type_id = float(self.get_parameter("unknown_type_id").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        cfg = PromptConfig()
        self.logic = EstimationLogic(cfg, api_key=self._get_api_key())

        self.bridge = self._make_bridge()

        self.create_service(PerceptionToEstimationRawImage, "perception_raw", self._on_raw)
        self.create_service(PerceptionToEstimationVisImage, "perception_vis", self._on_vis)
        self.create_service(PerceptionToEstimationTrashPoints, "perception_trash", self._on_trash)
        self.create_service(PerceptionToEstimationBinPoints, "perception_bin", self._on_bin)
        self.ctrl_client = self.create_client(
            EstimationToControlTrashBinPoints, "estimation_to_control"
        )

        self._cache: Dict[str, _InputSet] = {}
        self._busy = False
        self._busy_lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)

        self.get_logger().info("[Est] Ready (service-based).")

    def shutdown(self):
        self._executor.shutdown(wait=False)
        self.destroy_node()

    # ---- service handlers ----
    def _on_raw(self, request, response):
        return self._cache_set(request.session_id, "raw", request, response)

    def _on_vis(self, request, response):
        return self._cache_set(request.session_id, "vis", request, response)

    def _on_trash(self, request, response):
        return self._cache_set(request.session_id, "trash", request, response)

    def _on_bin(self, request, response):
        return self._cache_set(request.session_id, "binp", request, response)

    def _cache_set(self, session_id: str, key: str, req, res):
        sid = session_id.strip() or "default"
        entry = self._cache.setdefault(sid, _InputSet())
        setattr(entry, key, req)
        res.success = True
        res.message = "ok"
        self._maybe_start_infer(sid)
        return res

    # ---- core flow ----
    def _maybe_start_infer(self, sid: str):
        entry = self._cache.get(sid)
        if not entry or not (entry.raw and entry.vis and entry.trash and entry.binp):
            return
        if self.drop_if_busy:
            with self._busy_lock:
                if self._busy:
                    return
                self._busy = True
        self._executor.submit(self._infer_and_send, sid, entry)

    def _infer_and_send(self, sid: str, entry: _InputSet):
        try:
            if not self._is_fresh(entry):
                self.get_logger().warn(f"[Est] stale session={sid}, drop")
                return
            if self.bridge is None:
                self.get_logger().error("[Est] CvBridge missing, drop")
                return

            raw_bgr = ros_image_to_bgr_numpy(self.bridge, entry.raw.image)
            vis_bgr = ros_image_to_bgr_numpy(self.bridge, entry.vis.image)
            raw_jpg = bgr_numpy_to_jpeg_bytes(raw_bgr, jpeg_quality=self.jpeg_quality)
            vis_jpg = bgr_numpy_to_jpeg_bytes(vis_bgr, jpeg_quality=self.jpeg_quality)
            if raw_jpg is None or vis_jpg is None:
                self.get_logger().error("[Est] bgr->jpeg failed, drop")
                return

            trash_list = list(entry.trash.trash_list)
            bin_list = list(entry.binp.bin_list)
            if not trash_list:
                self.get_logger().warn("[Est] empty trash list, drop")
                return

            trash_list.sort(key=lambda t: float(t.tmp_id))
            expected_cnt = len(trash_list)

            labels = self.logic.run_inference(
                images=[(raw_jpg, "image/jpeg"), (vis_jpg, "image/jpeg")],
                expected_cnt=expected_cnt,
                unknown_id=self.unknown_type_id,
            )

            out = self._build_trash_bin_points(trash_list, bin_list, labels)
            self._send_to_control(sid, out)
        finally:
            if self.drop_if_busy:
                with self._busy_lock:
                    self._busy = False
            self._cache.pop(sid, None)

    def _send_to_control(self, sid: str, items: List[TrashBinPoint]):
        if self.dry_run:
            preview = []
            for it in items:
                preview.append([
                    float(it.type_id),
                    float(it.x), float(it.y), float(it.z),
                    float(it.angle),
                    float(it.bin_x), float(it.bin_y),
                ])
            self.get_logger().info(
                f"[Est][DRY_RUN] session={sid} items={len(items)} data={preview}"
            )
            return
        if not self.ctrl_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("[Est] control service not ready, drop")
            return
        req = EstimationToControlTrashBinPoints.Request()
        req.trash_bin_list = items
        req.session_id = sid
        self.ctrl_client.call_async(req)
        self.get_logger().info(f"[Est] sent TrashBinPoints N={len(items)}")

    # ---- helpers ----
    def _is_fresh(self, entry: _InputSet) -> bool:
        now = self._now_sec()
        for req in (entry.raw, entry.vis, entry.trash, entry.binp):
            if req is None:
                return False
            stamp = self._stamp_sec(req.stamp)
            if stamp is not None and (now - stamp) > self.max_age_sec:
                return False
        return True

    def _stamp_sec(self, stamp_msg: TimeMsg) -> Optional[float]:
        if stamp_msg is None:
            return None
        try:
            t = Time.from_msg(stamp_msg)
            return float(t.nanoseconds) * 1e-9
        except Exception:
            return None

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _make_bridge(self):
        try:
            from cv_bridge import CvBridge
            return CvBridge()
        except Exception as e:
            self.get_logger().error(f"[Est] CvBridge not available: {e}")
            return None

    def _get_api_key(self) -> Optional[str]:
        import os
        try:
            from dotenv import find_dotenv, load_dotenv
            load_dotenv(find_dotenv(usecwd=True))
        except Exception:
            pass
        return os.getenv("GEMINI_API_KEY")

    def _build_trash_bin_points(
        self,
        trash_list: List[TrashPoint],
        bin_list: List[BinPoint],
        labels: List[float],
    ) -> List[TrashBinPoint]:
        out: List[TrashBinPoint] = []
        if not bin_list:
            bin_list = [BinPoint(x=0.0, y=0.0)]
        for i, tp in enumerate(trash_list):
            bp = bin_list[i] if i < len(bin_list) else bin_list[0]
            item = TrashBinPoint()
            item.type_id = float(labels[i]) if i < len(labels) else self.unknown_type_id
            item.x = float(tp.x)
            item.y = float(tp.y)
            item.z = float(tp.z)
            item.angle = float(tp.angle)
            item.bin_x = float(bp.x)
            item.bin_y = float(bp.y)
            out.append(item)
        return out


def run_node(args=None):
    rclpy.init(args=args)
    node = EstimationServiceApp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()
