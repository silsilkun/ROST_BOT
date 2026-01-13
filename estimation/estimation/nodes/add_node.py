# add_node.py

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# ---- Import validator ----
try:
    from estimation.utils.utils import validate_waste_coordinates_flat
except Exception as e:  # pragma: no cover
    validate_waste_coordinates_flat = None
    _IMPORT_ERR = e


@dataclass
class _StampedArray:
    data: List[float]
    stamp_sec: float  # node clock seconds


class EstimationAddNode(Node):
    """
    Role:
      - coords([tmp_id,x,y,z,angle]*N) + type_ids([type]*N)
      - -> pickup_commands([type,x,y,z,angle]*N)

    Policy (원샷 테스트 안정화):
      - stale / sync 실패 시 버퍼를 반드시 비운다
      - publish 성공 후에도 버퍼를 비운다
    """

    def __init__(self):
        super().__init__("estimation_add")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("coords_topic", "/perception/waste_coordinates"),
                ("type_topic", "/estimation/type_id"),
                ("output_topic", "/estimation/pickup_commands"),
                ("unknown_type_id", -1.0),
                ("max_age_sec", 0.5),
                ("sync_tolerance_sec", 0.2),
                ("log_throttle_sec", 2.0),
            ],
        )

        self.coords_topic = self.get_parameter("coords_topic").value
        self.type_topic = self.get_parameter("type_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.unknown_type_id = float(self.get_parameter("unknown_type_id").value)
        self.max_age_sec = float(self.get_parameter("max_age_sec").value)
        self.sync_tolerance_sec = float(self.get_parameter("sync_tolerance_sec").value)
        self.log_throttle_sec = float(self.get_parameter("log_throttle_sec").value)

        self._last_coords: Optional[_StampedArray] = None
        self._last_types: Optional[_StampedArray] = None
        self._last_warn_sec: float = 0.0

        self._sub_coords = self.create_subscription(
            Float32MultiArray, self.coords_topic, self._on_coords, 10
        )
        self._sub_types = self.create_subscription(
            Float32MultiArray, self.type_topic, self._on_types, 10
        )
        self._pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)

        if validate_waste_coordinates_flat is None:
            self.get_logger().error(
                f"[Add] validate_waste_coordinates_flat import failed: {_IMPORT_ERR}"
            )

        self.get_logger().info(
            f"[Add] Ready. coords={self.coords_topic}, types={self.type_topic} -> pub={self.output_topic}"
        )

    # -------------------------
    # Callbacks
    # -------------------------
    def _on_coords(self, msg: Float32MultiArray) -> None:
        now = self._now_sec()
        self.get_logger().info(
            f"[Add][DBG] on_coords now={now:.3f} len={len(msg.data)}"
        )
        self._last_coords = _StampedArray(list(msg.data), now)
        self._try_publish(now)

    def _on_types(self, msg: Float32MultiArray) -> None:
        now = self._now_sec()
        self.get_logger().info(
            f"[Add][DBG] on_types now={now:.3f} len={len(msg.data)}"
        )
        self._last_types = _StampedArray(list(msg.data), now)
        self._try_publish(now)

    # -------------------------
    # Core logic
    # -------------------------
    def _try_publish(self, now_sec: float) -> None:
        if validate_waste_coordinates_flat is None:
            self._warn_throttled(now_sec, "[Add] validator not available -> drop")
            return

        if self._last_coords is None or self._last_types is None:
            return

        coords = self._last_coords
        types = self._last_types

        self.get_logger().info(
            f"[Add][DBG] try_publish now={now_sec:.3f} "
            f"coords_stamp={coords.stamp_sec:.3f} "
            f"types_stamp={types.stamp_sec:.3f}"
        )

        # ---- 1) Age gate (stale) ----
        if (now_sec - coords.stamp_sec) > self.max_age_sec:
            self._warn_throttled(
                now_sec,
                f"[Add] stale coords age={now_sec - coords.stamp_sec:.3f}s -> drop"
            )
            self._last_coords = None
            return

        if (now_sec - types.stamp_sec) > self.max_age_sec:
            self._warn_throttled(
                now_sec,
                f"[Add] stale types age={now_sec - types.stamp_sec:.3f}s -> drop"
            )
            self._last_types = None
            return

        # ---- 2) Sync gate ----
        dt = abs(coords.stamp_sec - types.stamp_sec)
        if dt > self.sync_tolerance_sec:
            self._warn_throttled(
                now_sec,
                f"[Add] coords/types out-of-sync dt={dt:.3f}s -> drop"
            )
            if coords.stamp_sec < types.stamp_sec:
                self._last_coords = None
            else:
                self._last_types = None
            return

        coords_flat = coords.data
        type_ids = types.data

        # ---- 3) Validate coords ----
        ret = validate_waste_coordinates_flat(coords_flat)
        if isinstance(ret, tuple):
            ok, reason = ret
        else:
            ok = bool(ret)
            reason = "invalid coords"

        if not ok:
            self._warn_throttled(now_sec, f"[Add] invalid coords -> drop ({reason})")
            self._clear_buffers()
            return

        n = len(coords_flat) // 5
        if n <= 0 or len(type_ids) != n:
            self._warn_throttled(
                now_sec,
                f"[Add] length mismatch coords={len(coords_flat)} types={len(type_ids)}"
            )
            self._clear_buffers()
            return

        # ---- 4) Sanitize types ----
        cleaned_types: List[float] = []
        for t in type_ids:
            if t is None or (isinstance(t, float) and (math.isnan(t) or math.isinf(t))):
                cleaned_types.append(self.unknown_type_id)
            else:
                cleaned_types.append(float(t))

        # ---- 5) Pack output ----
        out: List[float] = []
        for i in range(n):
            base = 5 * i
            out.extend([
                cleaned_types[i],
                coords_flat[base + 1],
                coords_flat[base + 2],
                coords_flat[base + 3],
                coords_flat[base + 4],
            ])

        msg = Float32MultiArray()
        msg.data = out
        self._pub.publish(msg)

        self.get_logger().info(
            f"[Add] published pickup_commands len={len(out)}"
        )

        # ---- 6) One-shot safety: clear buffers ----
        self._clear_buffers()

    # -------------------------
    # Helpers
    # -------------------------
    def _clear_buffers(self) -> None:
        self._last_coords = None
        self._last_types = None

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _warn_throttled(self, now_sec: float, text: str) -> None:
        if (now_sec - self._last_warn_sec) >= self.log_throttle_sec:
            self._last_warn_sec = now_sec
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = EstimationAddNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()