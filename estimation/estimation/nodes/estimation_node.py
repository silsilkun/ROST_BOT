import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from estimation.utils.config import CATEGORIES
from estimation.utils.setup_functions import (
    select_roi,
    select_bin_positions,
    close_setup_window
)
from estimation.utils.camera_capture import (
    init_camera,
    stop_camera,
    capture_snapshot,
    capture_snapshot_and_depth,
    crop_to_roi,
    crop_to_bbox
)
from estimation.utils.gemini_functions_v2 import (
    init_gemini_client,
    check_objects_exist,
    select_target_object,
    classify_object
)
from estimation.utils.calibration import gemini_to_robot


class VisionPipelineNode(Node):

    def __init__(self):
        super().__init__('vision_pipeline_node')

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/rost_output',
            10
        )

        self.get_logger().info("Vision Pipeline Node Started")

        # ─────────────────────────────
        # 카메라 & Gemini 초기화
        # ─────────────────────────────
        self.cam = init_camera()
        self.gemini = init_gemini_client()

        # ─────────────────────────────
        # 1회 설정: ROI + Bin 위치
        # ─────────────────────────────
        frame = capture_snapshot(self.cam)

        self.roi = select_roi(frame)
        self.bins = select_bin_positions(frame)
        close_setup_window()

        if self.roi is None or self.bins is None:
            self.get_logger().error("초기 설정 실패 → 노드 종료")
            rclpy.shutdown()
            return

        self.cycle = 0

        # 타이머 루프 (0.5초 주기)
        self.timer = self.create_timer(0.5, self.main_loop)


    def main_loop(self):
        self.cycle += 1
        self.get_logger().info(f"── Cycle #{self.cycle} ──")

        # RGB + Depth 캡처
        frame, depth_m = capture_snapshot_and_depth(self.cam)
        roi_img = crop_to_roi(frame, self.roi)

        # Step 1: 물체 존재 확인
        if not check_objects_exist(self.gemini, roi_img):
            self.get_logger().info("✅ 분리수거 완료!")
            return

        # Step 2: 타겟 선정
        target = select_target_object(self.gemini, roi_img)
        if target is None:
            self.get_logger().warn("타겟 선정 실패 → 건너뜀")
            return

        # Step 3: 분류
        bbox_img = crop_to_bbox(roi_img, target["bbox"])
        type_id = classify_object(self.gemini, bbox_img)

        # Step 4: 좌표 변환
        coords = gemini_to_robot(
            target["center"],
            self.roi,
            depth_m
        )

        if coords is None:
            self.get_logger().warn("좌표 변환 실패 → 건너뜀")
            return

        tx, ty, tz = coords

        # Bin 위치
        cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
        bx, by = self.bins.get(cat_name, self.bins["unknown"])

        # 최종 Output
        output = [
            float(type_id),
            float(tx),
            float(ty),
            float(tz),
            float(target["angle"]),
            float(bx),
            float(by)
        ]

        self.get_logger().info(f"📦 output = {output} ({cat_name})")

        msg = Float32MultiArray()
        msg.data = output
        self.publisher_.publish(msg)


    def destroy_node(self):
        stop_camera(self.cam)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = VisionPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
