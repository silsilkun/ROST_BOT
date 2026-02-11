"""
R.O.S.T - 메인 파이프라인 (main_pipeline.py)  ※ 참조용

⚠️ 배포용이 아닙니다.
   수환님께 "함수들이 이 순서로 연결됩니다"를 보여주는 참조 코드.

Output: [type_id, tx, ty, t_angle, bx, by]
  - type_id: 0~6 카테고리
  - tx, ty: 로봇 좌표 (캘리브레이션)
  - t_angle: 그리퍼 접근 각도 (0~180°)
  - bx, by: 해당 카테고리 쓰레기통 위치
  ※ tz(depth)는 ToF → Control 직접 전달
"""

from config import CATEGORIES
from setup_functions import select_roi, select_bin_positions
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, crop_to_roi, crop_to_bbox)
from gemini_functions import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from calibration import load_transform_matrix, uv_to_robot_coords


def main():
    # ── 초기화 ─────────────────────────────────────
    pipeline = init_camera()
    gemini = init_gemini_client()
    # [수정 포인트] 캘리브레이션 파일 경로
    T = load_transform_matrix(filepath=None)

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
        type_id = classify_object(gemini, bbox_img)

        # 좌표 변환
        tx, ty = uv_to_robot_coords(target["center"], roi, T)

        # Bin 위치
        cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
        bx, by = bins.get(cat_name, bins["unknown"])

        # ── Output (6개) ──────────────────────────
        # [수정 포인트] output 형식 바뀌면 여기만
        output = [type_id, tx, ty, target["angle"], bx, by]
        print(f"📦 output={output}  ({cat_name})")

    # ── 정리 ───────────────────────────────────────
    stop_camera(pipeline)


if __name__ == "__main__":
    main()
