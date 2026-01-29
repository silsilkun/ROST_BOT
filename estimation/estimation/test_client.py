# test_client.py
from __future__ import annotations

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Image

from rost_interfaces.srv import (
    PerceptionToEstimationRawImage,
    PerceptionToEstimationVisImage,
    PerceptionToEstimationTrashPoints,
    PerceptionToEstimationBinPoints,
)
from rost_interfaces.msg import TrashPoint, BinPoint


class EstimationTestClient(Node):
    def __init__(self):
        super().__init__("estimation_test_client")
        self.cli_raw = self.create_client(PerceptionToEstimationRawImage, "perception_raw")
        self.cli_vis = self.create_client(PerceptionToEstimationVisImage, "perception_vis")
        self.cli_trash = self.create_client(PerceptionToEstimationTrashPoints, "perception_trash")
        self.cli_bin = self.create_client(PerceptionToEstimationBinPoints, "perception_bin")

        for cli, name in (
            (self.cli_raw, "perception_raw"),
            (self.cli_vis, "perception_vis"),
            (self.cli_trash, "perception_trash"),
            (self.cli_bin, "perception_bin"),
        ):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"waiting for service: {name}")

    def send_dummy(self, trash_n: int = 2, bin_n: int = 2) -> None:
        session_id = str(self.get_clock().now().nanoseconds)
        stamp = self.get_clock().now().to_msg()

        raw = self._make_dummy_image(PerceptionToEstimationRawImage, stamp, session_id)
        vis = self._make_dummy_image(PerceptionToEstimationVisImage, stamp, session_id)
        trash = self._make_dummy_trash(stamp, session_id, trash_n)
        bins = self._make_dummy_bins(stamp, session_id, bin_n)

        futs = [
            self.cli_raw.call_async(raw),
            self.cli_vis.call_async(vis),
            self.cli_trash.call_async(trash),
            self.cli_bin.call_async(bins),
        ]

        for f in futs:
            rclpy.spin_until_future_complete(self, f)
            if f.result() is not None:
                self.get_logger().info(f"response: {f.result().success} {f.result().message}")
            else:
                self.get_logger().error("service call failed")

    def _make_dummy_image(self, srv_type, stamp: TimeMsg, session_id: str):
        msg = Image()
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = 1
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = 3
        msg.data = b"\x00\x00\x00"

        req = srv_type.Request()
        req.image = msg
        req.session_id = session_id
        req.stamp = stamp
        return req

    def _make_dummy_trash(self, stamp: TimeMsg, session_id: str, n: int):
        items = []
        for i in range(n):
            tp = TrashPoint()
            tp.tmp_id = float(i)
            tp.x = float(i) * 0.1
            tp.y = float(i) * 0.2
            tp.z = 0.0
            tp.angle = 0.0
            items.append(tp)
        req = PerceptionToEstimationTrashPoints.Request()
        req.trash_list = items
        req.session_id = session_id
        req.stamp = stamp
        return req

    def _make_dummy_bins(self, stamp: TimeMsg, session_id: str, n: int):
        items = []
        for i in range(n):
            bp = BinPoint()
            bp.x = float(i)
            bp.y = float(i)
            items.append(bp)
        req = PerceptionToEstimationBinPoints.Request()
        req.bin_list = items
        req.session_id = session_id
        req.stamp = stamp
        return req


def main(args=None):
    rclpy.init(args=args)
    node = EstimationTestClient()
    try:
        node.send_dummy()
    finally:
        node.destroy_node()
        rclpy.shutdown()
