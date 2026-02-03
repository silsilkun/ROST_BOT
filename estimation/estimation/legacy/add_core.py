# add_core.py
import math

from std_msgs.msg import Float32MultiArray
from estimation.utils.utils import validate_waste_coordinates_flat


class AddCore:
    def __init__(self, node, pub, unknown_type_id, max_age_sec, sync_tolerance_sec,
                 allow_stale_coords, log_throttle_sec):
        self.node = node
        self.pub = pub
        self.unknown_type_id = float(unknown_type_id)
        self.max_age_sec = float(max_age_sec)
        self.sync_tolerance_sec = float(sync_tolerance_sec)
        self.allow_stale_coords = bool(allow_stale_coords)
        self.log_throttle_sec = float(log_throttle_sec)
        self._last_coords = None  # (data, stamp)
        self._last_types = None   # (data, stamp)
        self._last_warn_sec = 0.0

    def on_coords(self, msg):
        now = self._now_sec()
        self._last_coords = (list(msg.data), now)
        self._try_publish(now)

    def on_types(self, msg):
        now = self._now_sec()
        self._last_types = (list(msg.data), now)
        self._try_publish(now)

    def _try_publish(self, now_sec):
        if validate_waste_coordinates_flat is None:
            self._warn(now_sec, "[Add] validator not available -> drop")
            return
        if self._last_coords is None or self._last_types is None:
            return

        coords, c_t = self._last_coords
        types, t_t = self._last_types

        if not self.allow_stale_coords:
            if (now_sec - c_t) > self.max_age_sec:
                self._warn(now_sec, f"[Add] stale coords age={now_sec - c_t:.3f}s -> drop")
                self._last_coords = None
                return
            if (now_sec - t_t) > self.max_age_sec:
                self._warn(now_sec, f"[Add] stale types age={now_sec - t_t:.3f}s -> drop")
                self._last_types = None
                return
            dt = abs(c_t - t_t)
            if dt > self.sync_tolerance_sec:
                self._warn(now_sec, f"[Add] coords/types out-of-sync dt={dt:.3f}s -> drop")
                if c_t < t_t:
                    self._last_coords = None
                else:
                    self._last_types = None
                return

        if not validate_waste_coordinates_flat(coords):
            self._warn(now_sec, "[Add] invalid coords -> drop")
            self._clear()
            return

        n = len(coords) // 5
        if n <= 0 or len(types) != n:
            self._warn(now_sec, f"[Add] length mismatch coords={len(coords)} types={len(types)}")
            self._clear()
            return

        cleaned = []
        for t in types:
            if t is None or (isinstance(t, float) and (math.isnan(t) or math.isinf(t))):
                cleaned.append(self.unknown_type_id)
            else:
                cleaned.append(float(t))

        out = []
        for i in range(n):
            base = 5 * i
            out.extend([cleaned[i], coords[base + 1], coords[base + 2], coords[base + 3], coords[base + 4]])

        msg = Float32MultiArray()
        msg.data = out
        self.pub.publish(msg)
        self.node.get_logger().info(f"[Add] published pickup_commands len={len(out)}")
        self._clear()

    def _clear(self):
        self._last_coords = None
        self._last_types = None

    def _now_sec(self):
        return float(self.node.get_clock().now().nanoseconds) * 1e-9

    def _warn(self, now_sec, text):
        if (now_sec - self._last_warn_sec) >= self.log_throttle_sec:
            self._last_warn_sec = now_sec
            self.node.get_logger().warn(text)
