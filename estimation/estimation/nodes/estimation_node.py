import rclpy
from rclpy.node import Node
from rost_interfaces.srv import EstimationToControl

# 기존 import 그대로 유지
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

        # 🔥 Service Client 생성
        self.client = self.create_client(
            EstimationToControl,
            'estimation_to_control'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for service...')

        self.get_logger().info("Vision Pipeline Node Started")

        # 카메라 & Gemini 초기화
        self.cam = init_camera()
        self.gemini = init_gemini_client()

        frame = capture_snapshot(self.cam)

        self.roi = select_roi(frame)
        self.bins = select_bin_positions(frame)
        close_setup_window()

        if self.roi is None or self.bins is None:
            self.get_logger().error("초기 설정 실패 → 노드 종료")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.5, self.main_loop)


    def main_loop(self):

        frame, depth_m = capture_snapshot_and_depth(self.cam)
        roi_img = crop_to_roi(frame, self.roi)

        if not check_objects_exist(self.gemini, roi_img):
            self.get_logger().info("✅ 분리수거 완료!")
            return

        target = select_target_object(self.gemini, roi_img)
        if target is None:
            return

        bbox_img = crop_to_bbox(roi_img, target["bbox"])
        type_id = classify_object(self.gemini, bbox_img)

        coords = gemini_to_robot(
            target["center"],
            self.roi,
            depth_m
        )

        if coords is None:
            self.get_logger().error("❌ 좌표 변환 실패: 안전을 위해 노드를 종료합니다.")
            self.timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()
            return

        tx, ty, tz = coords

        cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
        bx, by = self.bins.get(cat_name, self.bins["unknown"])

        output = [
            float(type_id),
            float(tx),
            float(ty),
            float(tz),
            float(target["angle"]),
            float(bx),
            float(by)
        ]

        self.get_logger().info(f"📤 Sending output: {output}")

        # 🔥 Service 요청 생성
        req = EstimationToControl.Request()
        req.data = output

        future = self.client.call_async(req)
        future.add_done_callback(self.service_callback)


    def service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"📦 Server response: {response.success}, {response.message}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


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
        if rclpy.ok():
            rclpy.shutdown()
