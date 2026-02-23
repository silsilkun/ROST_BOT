"""
R.O.S.T - 메인 파이프라인 (main_pipeline.py)  ※ 참조용

⚠️ 배포용이 아닙니다.
   수환님께 "함수들이 이 순서로 연결됩니다"를 보여주는 참조 코드.

Output: [type_id, tx, ty, short_side_px, t_angle, bx, by]
  - tx, ty 단위: mm
  - type_id: 0~6 카테고리
  - tx, ty: 로봇 좌표 (캘리브레이션)
  - t_angle: 그리퍼 접근 각도 (0~180°)
  - short_side_px: bbox 짧은 변 길이 (픽셀, ROI 기준)
  - bx, by: 해당 카테고리 쓰레기통 위치
  ※ tz(depth)는 ToF → Control 직접 전달
"""

import os
from config import (
    CATEGORIES, GEMINI_COORD_RANGE, PREFER_GRASP_PTS_SHORT_SIDE,
    GRIPPER_ANGLE_USE_GRASP_NORMAL, GRIPPER_ANGLE_OFFSET_DEG_CW,
    USE_COMPLEMENTARY_GRIPPER_ANGLE,
)
from setup_functions import select_roi, select_bin_positions
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, crop_to_roi, crop_to_bbox)
from gemini_functions import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from calibration import load_transform_matrix, uv_to_robot_coords, pixel_to_robot
import numpy as np


def main():
    # ── 초기화 ─────────────────────────────────────
    pipeline = init_camera()
    gemini = init_gemini_client()
    # [수정 포인트] 캘리브레이션 파일 경로
    calib_path = os.path.join(os.path.dirname(__file__), "utils", "camcalib.npz")
    T = load_transform_matrix(filepath=calib_path)

    # ── 1회 설정: ROI + Bin ────────────────────────
    frame = capture_snapshot(pipeline)
    roi = select_roi(frame)
    bins = select_bin_positions(frame)
    if roi is None or bins is None:
        print("초기 설정 실패 → 종료"); return

    # ── 메인 루프 ──────────────────────────────────
    cycle = 0
    while True:
        cycle += 1
        print(f"\n── Cycle #{cycle} ──")

        roi_img = crop_to_roi(capture_snapshot(pipeline), roi)

        # Step 1: 쓰레기 남아있어?
        if not check_objects_exist(gemini, roi_img):
            print("✅ 분리수거 완료!"); break

        # Step 2: 타겟 선정
        target = select_target_object(gemini, roi_img)
        if target is None:
            print("[건너뜀] 타겟 선정 실패"); continue

        # Step 3: 분류
        bbox_img = crop_to_bbox(roi_img, target["bbox"])
        type_id = classify_object(gemini, bbox_img, label_hint=target.get("label", ""))

        # 좌표 변환
        tx, ty = uv_to_robot_coords(target["center"], roi, T)

        # 짧은변 (픽셀, ROI 기준) + 대표 선분(norm) 추출
        seg_norm = None
        seg_is_grasp = False
        short_side_from_target = target.get("short_side_px")
        grasp_pts = target.get("grasp_pts")
        roi_h, roi_w = roi_img.shape[:2]
        px = lambda val, size: int(round(val / 1000 * size))

        if (isinstance(short_side_from_target, (int, float)) and short_side_from_target > 0 and
                isinstance(grasp_pts, list) and len(grasp_pts) == 2):
            short_side_px = int(round(short_side_from_target))
            seg_norm = (grasp_pts[0], grasp_pts[1])
            seg_is_grasp = True
        else:
            corners = target["corners"]
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

        # 짧은변 선분을 로봇좌표계(mm)로 변환해 길이/각도 산출
        roi_x, roi_y, roi_w, roi_h = roi
        if seg_norm is None:
            short_side_mm = short_side_px
            angle_out = float(target["angle"])
        else:
            (sx1, sy1), (sx2, sy2) = seg_norm  # norm [x,y]
            u1 = roi_x + px(sx1, roi_w)
            v1 = roi_y + px(sy1, roi_h)
            u2 = roi_x + px(sx2, roi_w)
            v2 = roi_y + px(sy2, roi_h)

            tx1, ty1 = pixel_to_robot(u1, v1, T)
            tx2, ty2 = pixel_to_robot(u2, v2, T)
            vx, vy = (tx2 - tx1), (ty2 - ty1)
            short_side_mm = int(round((vx ** 2 + vy ** 2) ** 0.5))

            angle_out = (np.degrees(np.arctan2(vy, vx)) + 180.0) % 180.0
            if seg_is_grasp and GRIPPER_ANGLE_USE_GRASP_NORMAL:
                angle_out = (angle_out + 90.0) % 180.0
            if USE_COMPLEMENTARY_GRIPPER_ANGLE:
                angle_out = (180.0 - angle_out) % 180.0
            angle_out = (angle_out + GRIPPER_ANGLE_OFFSET_DEG_CW) % 180.0

        # 요청 반영: 로봇 집기 가능 각도로 90도 시프트
        if 0.0 <= angle_out < 90.0:
            angle_out += 90.0
        else:
            angle_out -= 90.0
        angle_out = angle_out % 180.0

        # Bin 위치
        cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
        bx, by = bins.get(cat_name, bins["unknown"])

        # ── Output (7개) ──────────────────────────
        # [수정 포인트] output 형식 바뀌면 여기만
        output = [type_id, tx, ty, short_side_mm, float(angle_out), bx, by]
        print(f"📦 output={output}  ({cat_name})")

    # ── 정리 ───────────────────────────────────────
    stop_camera(pipeline)


if __name__ == "__main__":
    main()
