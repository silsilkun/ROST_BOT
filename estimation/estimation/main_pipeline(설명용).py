"""
R.O.S.T - 메인 파이프라인 (main_pipeline.py)  ※ 참조용

⚠️ 배포용이 아닙니다.
   수환님께 "함수들이 이 순서로 연결됩니다"를 보여주는 참조 코드.

Output: [type_id, tx, ty, tz, t_angle, bx, by]
  - type_id: 0~6 카테고리
  - tx, ty, tz: 로봇 작업 좌표 (cm, 캘리브레이션 변환)
  - t_angle: 그리퍼 접근 각도 (0~180°)
  - bx, by: 해당 카테고리 쓰레기통 위치

  ※ tz는 RealSense depth + 캘리브레이션으로 계산
  ※ ToF 센서는 Control이 별도 구독 (여기서 사용 안 함)
"""

from config import CATEGORIES
from setup_functions import select_roi, select_bin_positions, close_setup_window
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, capture_snapshot_and_depth,
                            crop_to_roi, crop_to_bbox)
from gemini_functions_v2 import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from calibration import gemini_to_robot


def main():
    # ── 초기화 ─────────────────────────────────────
    cam = init_camera()        # (pipeline, align) 튜플
    gemini = init_gemini_client()

    # ── 1회 설정: ROI + Bin ────────────────────────
    frame = capture_snapshot(cam)
    roi = select_roi(frame)
    bins = select_bin_positions(frame)
    close_setup_window()
    if roi is None or bins is None:
        print("초기 설정 실패 → 종료"); return

    # ── 메인 루프 ──────────────────────────────────
    cycle = 0
    while True:
        cycle += 1
        print(f"\n── Cycle #{cycle} ──")

        # RGB + Depth 동시 캡처
        frame, depth_m = capture_snapshot_and_depth(cam)
        roi_img = crop_to_roi(frame, roi)

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

        # 좌표 변환: Gemini → 로봇 좌표 (RealSense depth 사용)
        coords = gemini_to_robot(target["center"], roi, depth_m)
        if coords is None:
            print("[건너뜀] 좌표 변환 실패"); continue
        tx, ty, tz = coords

        # Bin 위치
        cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
        bx, by = bins.get(cat_name, bins["unknown"])

        # ── Output (7개) ──────────────────────────
        # [수정 포인트] output 형식 바뀌면 여기만
        output = [type_id, tx, ty, tz, target["angle"], bx, by]
        print(f"📦 output={output}  ({cat_name})")

    # ── 정리 ───────────────────────────────────────
    stop_camera(cam)


if __name__ == "__main__":
    main()
