"""
R.O.S.T - 통합 테스트 (test_pipeline.py)
노드 없이 기능 함수들만 순서대로 테스트한다.

실행 방법:
  1) .env 파일에 API 키 넣기:  GEMINI_API_KEY=your_key_here
  2) pip install google-genai pyrealsense2 python-dotenv opencv-python
  3) python test_pipeline.py

현재 상태:
  ✅ RealSense 카메라 — 연결됨
  ❌ ToF 센서 — 미도착 → 더미값(250mm) 사용
  ❌ Calibration — 파트장님 복귀 후 → 단위행렬 사용 (uv 그대로 나옴)
"""

import os
import sys
import cv2
import numpy as np

# Qt 백엔드 윈도우 스레드 시작 (이게 없으면 setMouseCallback에서 NULL pointer 발생)
cv2.startWindowThread()

# ── .env 로드 ──────────────────────────────────────────
# [수정 포인트] .env 파일 위치가 다르면 여기만 수정
from dotenv import load_dotenv
load_dotenv()  # 현재 디렉토리의 .env 읽기

# [안전장치] API 키 확인
if not os.environ.get("GEMINI_API_KEY"):
    print("❌ .env 파일에 GEMINI_API_KEY가 없습니다!")
    print("   .env 파일 예시: GEMINI_API_KEY=AIzaSy...")
    sys.exit(1)

# ── 모듈 import ────────────────────────────────────────
from config import CATEGORIES
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, crop_to_roi, crop_to_bbox)
from setup_functions import select_roi, select_bin_positions
from gemini_functions import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from calibration import load_transform_matrix, uv_to_robot_coords


# ── 안전한 imshow (Qt 백엔드 에러 방지) ────────────────
def safe_imshow(title: str, image):
    """Qt 백엔드 호환 imshow"""
    cv2.imshow(title, image)
    cv2.waitKey(1)


# ── ToF 더미 함수 (센서 도착 전까지 사용) ──────────────
def read_depth_dummy() -> float:
    """
    ToF 센서가 없으므로 고정값 반환.
    [수정 포인트] 센서 도착하면 이 함수 대신 tof_sensor.read_depth_stable() 사용
    """
    DUMMY_DEPTH = 250.0  # mm
    print(f"[ToF 더미] depth = {DUMMY_DEPTH}mm (센서 미연결)")
    return DUMMY_DEPTH


# ── 테스트 메뉴 ────────────────────────────────────────
def print_menu():
    print("\n" + "=" * 50)
    print("  R.O.S.T 기능 테스트 메뉴")
    print("=" * 50)
    print("  1) 카메라 테스트        — 스냅샷 촬영 확인")
    print("  2) ROI 선택 테스트      — 마우스 드래그")
    print("  3) Gemini Step 1 테스트 — 객체 존재 확인")
    print("  4) Gemini Step 2 테스트 — 타겟 선정")
    print("  5) Gemini Step 3 테스트 — 카테고리 분류")
    print("  6) 전체 1사이클 테스트   — 1~5 한 번에 실행")
    print("  7) 전체 루프 테스트      — 쓰레기 소진까지 반복")
    print("  q) 종료")
    print("-" * 50)


# ── 개별 테스트 함수들 ─────────────────────────────────

def test_camera(pipeline):
    """카메라 스냅샷 촬영 + 화면 표시"""
    print("\n[테스트] 카메라 스냅샷...")
    frame = capture_snapshot(pipeline)
    print(f"  shape: {frame.shape}, dtype: {frame.dtype}")
    safe_imshow("Camera Test", frame)
    print("  → 아무 키나 누르면 닫힘")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return frame


def test_roi(pipeline):
    """ROI 선택 테스트"""
    print("\n[테스트] ROI 선택...")
    frame = capture_snapshot(pipeline)
    roi = select_roi(frame)
    if roi is None:
        print("  ❌ ROI 선택 안 됨")
        return None, None

    # ROI 영역 시각화
    x, y, w, h = roi
    display = frame.copy()
    cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)
    safe_imshow("ROI Result", display)
    print("  → 아무 키나 누르면 닫힘")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return frame, roi


def test_step1(gemini, pipeline, roi):
    """Gemini Step 1: 객체 존재 확인"""
    print("\n[테스트] Step 1 — 객체 존재 확인...")
    frame = capture_snapshot(pipeline)
    roi_img = crop_to_roi(frame, roi)

    safe_imshow("ROI Image (Step 1 Input)", roi_img)
    cv2.waitKey(1)

    result = check_objects_exist(gemini, roi_img)
    print(f"  결과: {'쓰레기 있음 ✓' if result else '비어있음 ✗'}")
    cv2.destroyAllWindows()
    return result, roi_img


def test_step2(gemini, roi_img):
    """Gemini Step 2: 타겟 선정"""
    print("\n[테스트] Step 2 — 타겟 선정...")
    target = select_target_object(gemini, roi_img)

    if target is None:
        print("  ❌ 타겟 선정 실패")
        return None

    # bbox 시각화
    h, w = roi_img.shape[:2]
    ymin, xmin, ymax, xmax = target["bbox"]
    # 정규화(0~1000) → 픽셀
    px = lambda val, size: int(val / 1000 * size)
    p1 = (px(xmin, w), px(ymin, h))
    p2 = (px(xmax, w), px(ymax, h))
    cy, cx = target["center"]
    center_px = (px(cx, w), px(cy, h))

    display = roi_img.copy()
    cv2.rectangle(display, p1, p2, (0, 255, 0), 2)
    cv2.circle(display, center_px, 5, (0, 0, 255), -1)
    cv2.putText(display, f"{target['label']} ({target['angle']:.0f}deg)",
                (p1[0], p1[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    safe_imshow("Step 2: Target", display)
    print("  → 아무 키나 누르면 닫힘")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return target


def test_step3(gemini, roi_img, target):
    """Gemini Step 3: 카테고리 분류"""
    print("\n[테스트] Step 3 — 카테고리 분류...")
    bbox_img = crop_to_bbox(roi_img, target["bbox"])

    safe_imshow("Step 3: Cropped Object", bbox_img)
    cv2.waitKey(1)

    type_id = classify_object(gemini, bbox_img)
    cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
    print(f"  결과: {cat_name} (type_id={type_id})")
    cv2.destroyAllWindows()
    return type_id


def test_full_cycle(gemini, pipeline, roi, T):
    """1사이클 전체 테스트 (한 개 객체 처리)"""
    print("\n" + "─" * 50)
    print("  전체 1사이클 테스트")
    print("─" * 50)

    # Step 1
    has_obj, roi_img = test_step1(gemini, pipeline, roi)
    if not has_obj:
        print("  → 객체 없음, 사이클 종료")
        return None

    # Step 2
    target = test_step2(gemini, roi_img)
    if target is None:
        return None

    # Step 3
    type_id = test_step3(gemini, roi_img, target)

    # Depth (더미)
    tz = read_depth_dummy()

    # 좌표 변환 (placeholder)
    tx, ty = uv_to_robot_coords(target["center"], roi, T)

    # Output 조립
    cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
    # [수정 포인트] bin 좌표는 테스트에서는 더미값 사용
    bx, by = 0.0, 0.0
    output = [type_id, tx, ty, tz, target["angle"], bx, by]

    print(f"\n  📦 Output: {output}")
    print(f"     분류: {cat_name}")
    print(f"     좌표: tx={tx:.2f}, ty={ty:.2f}, tz={tz:.1f}")
    print(f"     각도: {target['angle']}°")
    print(f"     bin:  ({bx}, {by}) ← 테스트 더미값")
    return output


def test_full_loop(gemini, pipeline, roi, T):
    """루프 테스트 (객체 소진까지 반복)"""
    print("\n" + "=" * 50)
    print("  전체 루프 테스트 시작")
    print("  → q 키로 중간 종료 가능")
    print("=" * 50)

    cycle = 0
    while True:
        cycle += 1
        print(f"\n{'━' * 40}")
        print(f"  Cycle #{cycle}")
        print(f"{'━' * 40}")

        result = test_full_cycle(gemini, pipeline, roi, T)
        if result is None:
            print("\n✅ 루프 종료!")
            break

        print("\n  [다음 사이클] 아무 키 = 계속 / q = 종료")
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        if key == ord('q'):
            print("  → 사용자 종료")
            break

    print(f"\n총 {cycle}회 사이클 실행 완료")


# ── 메인 ───────────────────────────────────────────────

def main():
    print("R.O.S.T 기능 테스트 시작\n")

    # 초기화
    pipeline = init_camera()
    gemini = init_gemini_client()
    T = load_transform_matrix()  # placeholder 단위행렬

    # 상태 저장
    roi = None

    while True:
        print_menu()
        # "1)", "1.", "1" 전부 허용
        choice = input("선택: ").strip().lower().rstrip(").")

        if choice == "1":
            test_camera(pipeline)

        elif choice == "2":
            _, roi = test_roi(pipeline)

        elif choice == "3":
            if roi is None:
                print("⚠️  ROI를 먼저 선택하세요 (메뉴 2)")
                continue
            test_step1(gemini, pipeline, roi)

        elif choice == "4":
            if roi is None:
                print("⚠️  ROI를 먼저 선택하세요 (메뉴 2)")
                continue
            frame = capture_snapshot(pipeline)
            roi_img = crop_to_roi(frame, roi)
            test_step2(gemini, roi_img)

        elif choice == "5":
            if roi is None:
                print("⚠️  ROI를 먼저 선택하세요 (메뉴 2)")
                continue
            frame = capture_snapshot(pipeline)
            roi_img = crop_to_roi(frame, roi)
            target = select_target_object(gemini, roi_img)
            if target:
                test_step3(gemini, roi_img, target)

        elif choice == "6":
            if roi is None:
                print("⚠️  ROI를 먼저 선택하세요 (메뉴 2)")
                continue
            test_full_cycle(gemini, pipeline, roi, T)

        elif choice == "7":
            if roi is None:
                print("⚠️  ROI를 먼저 선택하세요 (메뉴 2)")
                continue
            test_full_loop(gemini, pipeline, roi, T)

        elif choice == "q":
            break

        else:
            print("잘못된 입력")

    # 정리
    stop_camera(pipeline)
    cv2.destroyAllWindows()
    print("\n테스트 종료")


if __name__ == "__main__":
    main()
