"""
R.O.S.T - 메인 파이프라인 (설명용)

ROS2 노드 없이 함수 연결 순서를 설명하기 위한 참고 코드.
실운영은 estimation.nodes.estimation_node 사용.
"""

from estimation.utils.config import CATEGORIES, GEMINI_COORD_RANGE
from estimation.utils.setup_functions import (
    select_roi,
    select_bin_positions,
    close_setup_window,
)
from estimation.utils.camera_capture import (
    init_camera,
    stop_camera,
    capture_snapshot,
    capture_snapshot_and_depth,
    crop_to_roi,
    crop_to_bbox,
)
from estimation.utils.gemini_functions_v2 import (
    init_gemini_client,
    check_objects_exist,
    select_target_object,
    classify_object,
)
from estimation.utils.calibration import gemini_to_robot


def main():
    cam = init_camera()
    gemini = init_gemini_client()

    frame = capture_snapshot(cam)
    roi = select_roi(frame)
    bins = select_bin_positions(frame)
    close_setup_window()

    if roi is None or bins is None:
        print("초기 설정 실패 -> 종료")
        stop_camera(cam)
        return

    cycle = 0
    while True:
        cycle += 1
        print(f"\n── Cycle #{cycle} ──")

        frame, depth_m = capture_snapshot_and_depth(cam)
        roi_img = crop_to_roi(frame, roi)

        if not check_objects_exist(gemini, roi_img):
            print("✅ 분리수거 완료!")
            continue

        target = select_target_object(gemini, roi_img)
        if target is None:
            continue

        bbox_img = crop_to_bbox(roi_img, target["bbox"])
        type_id = classify_object(gemini, bbox_img)

        coords = gemini_to_robot(target["center"], roi, depth_m)
        if coords is None:
            print("❌ 좌표 변환 실패 -> 안전 중지")
            break
        tx, ty, _tz = coords

        cat_name = next((k for k, v in CATEGORIES.items() if v == type_id), "unknown")
        bx, by = bins.get(cat_name, bins["unknown"])

        ymin, xmin, ymax, xmax = target["bbox"]
        roi_h, roi_w = roi_img.shape[:2]
        bbox_w = (xmax - xmin) / GEMINI_COORD_RANGE * roi_w
        bbox_h = (ymax - ymin) / GEMINI_COORD_RANGE * roi_h
        short_side_px = int(round(min(bbox_w, bbox_h)))

        # output format:
        # [type_id, tx, ty, short_side_px, angle, bx, by]
        output = [
            float(type_id),
            float(tx),
            float(ty),
            float(short_side_px),
            float(target["angle"]),
            float(bx),
            float(by),
        ]
        print(f"📦 output={output} ({cat_name}, short_side_px={short_side_px})")


if __name__ == "__main__":
    main()
