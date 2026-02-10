"""
R.O.S.T - 메인 파이프라인 (main_pipeline.py)  ※ 참조용

⚠️ 배포용이 아닙니다.
   수환님께 "함수들이 이 순서로 연결됩니다"를 보여주는 참조 코드.
   실제 노드 구성/통신은 수환님이 담당.
"""

from config import CATEGORIES
from setup_functions import select_roi, select_bin_positions
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, crop_to_roi, crop_to_bbox)
from gemini_functions import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from tof_sensor import init_tof_sensor, close_tof_sensor, read_depth_stable
from calibration import load_transform_matrix, uv_to_robot_coords


def main():
    # ── 초기화 ─────────────────────────────────────
    pipeline = init_camera()
    gemini = init_gemini_client()
    tof = init_tof_sensor()
    # [수정 포인트] 캘리브레이션 파일 경로 → 파트장님 데이터로 교체
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

        # 스냅샷 + ROI 크롭
        roi_img = crop_to_roi(capture_snapshot(pipeline), roi)

        # Step 1: 쓰레기 남아있어?
        if not check_objects_exist(gemini, roi_img):
            print("✅ 분리수거 완료!"); break

        # Step 2: 가장 집기 쉬운 객체 선정
        target = select_target_object(gemini, roi_img)
        if target is None:
            print("[건너뜀] 타겟 선정 실패"); continue

        # Step 3: 카테고리 분류 (bbox 확대)
        bbox_img = crop_to_bbox(roi_img, target["bbox"])
        type_id = classify_object(gemini, bbox_img)

        # Depth 측정 (ToF)
        tz = read_depth_stable(tof)

        # 좌표 변환: Gemini uv → 로봇 좌표
        tx, ty = uv_to_robot_coords(target["center"], roi, T)

        # Bin 위치 가져오기
        cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
        bx, by = bins.get(cat_name, bins["unknown"])

        # ── 최종 Output (7개 값) ──────────────────
        # [수정 포인트] output 형식이 바뀌면 여기만 수정
        output = [type_id, tx, ty, tz, target["angle"], bx, by]
        print(f"📦 output={output}  ({cat_name})")

        # → control 파트 전달은 수환님 통신 구조에 따라 연결
        # send_to_control(output)
        # wait_for_control_done()

    # ── 정리 ───────────────────────────────────────
    close_tof_sensor(tof)
    stop_camera(pipeline)


if __name__ == "__main__":
    main()
