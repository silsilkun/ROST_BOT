import numpy as np
import rclpy
from rclpy.node import Node
from rost_interfaces.srv import EstimationToControl

from estimation.utils.config import CATEGORIES
from estimation.utils.config import GEMINI_COORD_RANGE
from estimation.utils.config import PREFER_GRASP_PTS_SHORT_SIDE
from estimation.utils.config import GRIPPER_ANGLE_USE_GRASP_NORMAL
from estimation.utils.config import USE_COMPLEMENTARY_GRIPPER_ANGLE
from estimation.utils.config import GRIPPER_ANGLE_OFFSET_DEG_CW
from estimation.utils.config import BIN_POSITIONS
from estimation.utils.setup_functions import (
    select_roi,
    close_setup_window,
)
from estimation.utils.camera_capture import (
    init_camera,
    stop_camera,
    capture_snapshot,
    capture_snapshot_and_depth,
    crop_to_roi,
    crop_to_bbox
)
from ROST_BOT.estimation.estimation.utils.gemini_functions import (
    init_gemini_client,
    check_objects_exist,
    select_target_object,
    classify_object
)
from estimation.utils.calibration import gemini_to_robot, load_transform_matrix, pixel_to_robot


class VisionPipelineNode(Node):

    def __init__(self):
        super().__init__('vision_pipeline_node')

        # 🔥 Service Client 생성
        self.client = self.create_client(
            EstimationToControl,
            'estimation_to_control'
        )

        if self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Control service connected.")
        else:
            self.get_logger().warn("Control service not available. Running estimation-only mode.")

        self.get_logger().info("Vision Pipeline Node Started")

        # 카메라 & Gemini 초기화
        self.cam = init_camera()
        self.gemini = init_gemini_client()

        self.calib = load_transform_matrix()
        frame, _depth = capture_snapshot(self.cam)

        self.roi = select_roi(frame)
        self.bins = {
            name: (float(pos[0]), float(pos[1]))
            for name, pos in BIN_POSITIONS.items()
        }
        close_setup_window()

        if self.roi is None:
            self.get_logger().error("초기 설정 실패 → 노드 종료")
            rclpy.shutdown()
            return
        if "unknown" not in self.bins:
            self.get_logger().error("BIN_POSITIONS에 'unknown' 키가 필요합니다.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Using fixed BIN_POSITIONS from config: {self.bins}")

        self.cycle = 0
        self.request_in_flight = False
        self.timer = self.create_timer(0.5, self.main_loop)

    def _compute_short_side_mm_and_angle(self, target, roi, roi_img, depth_m):
        short_side_from_target = target.get("short_side_px")
        grasp_pts = target.get("grasp_pts")
        corners = target.get("corners")
        roi_h, roi_w = roi_img.shape[:2]
        px = lambda val, size: int(round(val / GEMINI_COORD_RANGE * size))

        seg_norm = None
        seg_is_grasp = False

        if (isinstance(short_side_from_target, (int, float)) and short_side_from_target > 0 and
                isinstance(grasp_pts, list) and len(grasp_pts) == 2):
            short_side_px = int(round(short_side_from_target))
            seg_norm = (grasp_pts[0], grasp_pts[1])
            seg_is_grasp = True
        else:
            if not (isinstance(corners, list) and len(corners) == 4):
                return None, None
            pts = [(px(x, roi_w), px(y, roi_h)) for x, y in corners]
            def dist(a, b):
                return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
            edge_info = []
            for i in range(4):
                j = (i + 1) % 4
                d = dist(pts[i], pts[j])
                edge_info.append((d, i, j))
            short_side_from_corners, imin, jmin = min(edge_info, key=lambda x: x[0])
            short_side_px = int(round(short_side_from_corners))
            seg_norm = (corners[imin], corners[jmin])

            if PREFER_GRASP_PTS_SHORT_SIDE and isinstance(grasp_pts, list) and len(grasp_pts) == 2:
                (gx1, gy1), (gx2, gy2) = grasp_pts
                gp1 = (px(gx1, roi_w), px(gy1, roi_h))
                gp2 = (px(gx2, roi_w), px(gy2, roi_h))
                grasp_dist = dist(gp1, gp2)
                ratio = grasp_dist / max(1.0, short_side_from_corners)
                if grasp_dist >= 3.0 and 0.6 <= ratio <= 1.4:
                    short_side_px = int(round(grasp_dist))
                    seg_norm = (grasp_pts[0], grasp_pts[1])
                    seg_is_grasp = True

        if seg_norm is None:
            return short_side_px, None

        roi_x, roi_y, roi_w, roi_h = roi
        (sx1, sy1), (sx2, sy2) = seg_norm
        u1 = roi_x + px(sx1, roi_w)
        v1 = roi_y + px(sy1, roi_h)
        u2 = roi_x + px(sx2, roi_w)
        v2 = roi_y + px(sy2, roi_h)

        p1 = pixel_to_robot(u1, v1, depth_m, self.calib)
        p2 = pixel_to_robot(u2, v2, depth_m, self.calib)
        if p1 is None or p2 is None:
            return short_side_px, None
        tx1, ty1 = p1
        tx2, ty2 = p2
        vx, vy = (tx2 - tx1), (ty2 - ty1)
        dist_mm = (vx ** 2 + vy ** 2) ** 0.5
        if dist_mm <= 0.0:
            return short_side_px, None

        short_side_mm = int(round(dist_mm))
        # Use test_pipeline angle logic: abs-based short-side angle
        angle_out = np.degrees(np.arctan2(abs(vy), abs(vx)))
        if seg_is_grasp and GRIPPER_ANGLE_USE_GRASP_NORMAL:
            angle_out = (angle_out + 90.0) % 180.0
        if USE_COMPLEMENTARY_GRIPPER_ANGLE:
            angle_out = (180.0 - angle_out) % 180.0
        angle_out = (angle_out + GRIPPER_ANGLE_OFFSET_DEG_CW) % 180.0

        return short_side_mm, float(angle_out)

    def _short_side_mm_from_bbox(self, bbox, roi, roi_img, depth_m):
        """
        bbox: [ymin, xmin, ymax, xmax] in GEMINI_COORD_RANGE.
        Returns short side length in mm (robot coords) if possible, else None.
        """
        if bbox is None or depth_m is None:
            return None

        ymin, xmin, ymax, xmax = bbox
        roi_h, roi_w = roi_img.shape[:2]
        w_px = (xmax - xmin) / GEMINI_COORD_RANGE * roi_w
        h_px = (ymax - ymin) / GEMINI_COORD_RANGE * roi_h

        if w_px <= 0 or h_px <= 0:
            return None

        # Center of bbox (roi-local px)
        cx = (xmin + xmax) / 2.0 / GEMINI_COORD_RANGE * roi_w
        cy = (ymin + ymax) / 2.0 / GEMINI_COORD_RANGE * roi_h

        if w_px <= h_px:
            p1 = (cx - w_px / 2.0, cy)
            p2 = (cx + w_px / 2.0, cy)
        else:
            p1 = (cx, cy - h_px / 2.0)
            p2 = (cx, cy + h_px / 2.0)

        roi_x, roi_y, _rw, _rh = roi
        u1 = int(round(roi_x + p1[0]))
        v1 = int(round(roi_y + p1[1]))
        u2 = int(round(roi_x + p2[0]))
        v2 = int(round(roi_y + p2[1]))

        p1 = pixel_to_robot(u1, v1, depth_m, self.calib)
        p2 = pixel_to_robot(u2, v2, depth_m, self.calib)
        if p1 is None or p2 is None:
            return None
        tx1, ty1 = p1
        tx2, ty2 = p2

        dx, dy = (tx2 - tx1), (ty2 - ty1)
        dist_mm = (dx ** 2 + dy ** 2) ** 0.5
        if dist_mm <= 0.0:
            return None

        return int(round(dist_mm))


    def main_loop(self):
        # control 응답 전에 재요청이 쌓이면 동작이 겹치므로 차단한다.
        if self.request_in_flight:
            return

        self.cycle += 1
        self.get_logger().info(f"── Cycle #{self.cycle} ──")

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
            self.get_logger().warn("⚠️ depth 기반 좌표 변환 실패: 이번 사이클은 스킵하고 재시도합니다.")
            return

        tx, ty, _tz = coords

        cat_name = next((k for k, v in CATEGORIES.items() if v == type_id), "unknown")
        bx, by = self.bins.get(cat_name, self.bins["unknown"])

        # main_pipeline 기준: short_side를 로봇 좌표계(mm)로 변환
        short_side_mm, angle_out = self._compute_short_side_mm_and_angle(target, self.roi, roi_img, depth_m)
        if short_side_mm is None:
            short_side_mm = self._short_side_mm_from_bbox(target.get("bbox"), self.roi, roi_img, depth_m)
        if short_side_mm is None:
            ymin, xmin, ymax, xmax = target["bbox"]
            roi_h, roi_w = roi_img.shape[:2]
            bbox_w = (xmax - xmin) / GEMINI_COORD_RANGE * roi_w
            bbox_h = (ymax - ymin) / GEMINI_COORD_RANGE * roi_h
            short_side_mm = int(round(min(bbox_w, bbox_h)))
            self.get_logger().warn(
                "short_side_mm 변환 실패 → px 길이를 그대로 사용합니다."
            )
        if angle_out is None:
            angle_out = float(target["angle"])

        # output format:
        # [type_id, tx_mm, ty_mm, short_side_mm, angle_deg, bx_mm, by_mm]
        output = [
            float(type_id),
            float(tx),
            float(ty),
            float(short_side_mm),
            float(angle_out),
            float(bx),
            float(by),
        ]

        self.get_logger().info(
            f"📤 Sending output: {output} ({cat_name}, short_side_mm={short_side_mm})"
        )

        # 🔥 Service 요청 생성
        req = EstimationToControl.Request()
        req.data = output

        self.request_in_flight = True
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
        finally:
            self.request_in_flight = False


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
