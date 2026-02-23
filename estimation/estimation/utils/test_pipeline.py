"""
R.O.S.T - 통합 테스트 (test_pipeline.py)
노드 없이 기능 함수들만 순서대로 테스트한다.

실행 방법:
  1) .env 파일에 API 키 넣기:  GEMINI_API_KEY=your_key_here
  2) pip install google-genai pyrealsense2 python-dotenv opencv-python
  3) python test_pipeline.py

Output: [type_id, tx, ty, t_angle, short_side_px, bx, by]
  ※ tz(depth)는 ToF → Control 직접 전달 (이 테스트에 포함 안 됨)
"""

import os
import sys
import cv2
import numpy as np

# Qt 백엔드 윈도우 스레드 시작
cv2.startWindowThread()

# ── .env 로드 ──────────────────────────────────────────
from dotenv import load_dotenv
load_dotenv()

if not os.environ.get("GEMINI_API_KEY"):
    print("❌ .env 파일에 GEMINI_API_KEY가 없습니다!")
    print("   .env 파일 예시: GEMINI_API_KEY=AIzaSy...")
    sys.exit(1)

# ── 모듈 import ────────────────────────────────────────
from config import CATEGORIES, GEMINI_COORD_RANGE
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, crop_to_roi, crop_to_bbox)
from setup_functions import select_roi, select_bin_positions, close_setup_window
from gemini_functions_v2 import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from calibration import load_transform_matrix, uv_to_robot_coords


# ── 테스트 메뉴 ────────────────────────────────────────
def print_menu():
    print("\n" + "=" * 50)
    print("  R.O.S.T 기능 테스트 메뉴")
    print("=" * 50)
    print("  1) 카메라 테스트        — 스냅샷 촬영 확인")
    print("  2) 초기 설정            — ROI 선택 + Bin 위치 지정")
    print("  3) Gemini Step 1 테스트 — 객체 존재 확인")
    print("  4) Gemini Step 2 테스트 — 타겟 선정")
    print("  5) Gemini Step 3 테스트 — 카테고리 분류")
    print("  6) 전체 1사이클 테스트   — 3~5 한 번에 실행")
    print("  7) 전체 루프 테스트      — 쓰레기 소진까지 반복")
    print("  q) 종료")
    print("-" * 50)


def print_setup_status(roi, bins):
    """현재 설정 상태 표시"""
    roi_str = f"x={roi[0]}, y={roi[1]}, w={roi[2]}, h={roi[3]}" if roi else "미설정"
    bins_str = f"{len(bins)}개 설정됨" if bins else "미설정"
    print(f"  [현재 설정] ROI: {roi_str} | Bin: {bins_str}")


# ── 개별 테스트 함수들 ─────────────────────────────────

def test_camera(pipeline):
    """카메라 스냅샷 촬영 + 화면 표시"""
    print("\n[테스트] 카메라 스냅샷...")
    frame = capture_snapshot(pipeline)
    print(f"  shape: {frame.shape}, dtype: {frame.dtype}")
    cv2.imshow("Camera Test", frame)
    cv2.waitKey(1)
    print("  → 아무 키나 누르면 닫힘")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.waitKey(100)
    return frame


def test_setup(pipeline):
    """ROI 선택 + Bin 위치 지정 (하나의 창에서 연속 진행)"""
    print("\n[설정] 초기 설정을 시작합니다.")

    frame = capture_snapshot(pipeline)
    print(f"  스냅샷 촬영 완료 ({frame.shape[1]}x{frame.shape[0]})")

    # Step A: ROI 선택 (창이 여기서 열림)
    print("\n── ROI 선택 ──")
    roi = select_roi(frame)
    if roi is None:
        print("  ❌ ROI 선택 실패")
        close_setup_window()
        return None, None

    # Step B: Bin 위치 선택 (같은 창에서 계속)
    print("\n── Bin 위치 선택 ──")
    print("  카테고리별 쓰레기통 위치를 클릭합니다.")
    print("  type_id 매핑:")
    for cat, tid in CATEGORIES.items():
        print(f"    {tid}: {cat}")
    print()

    bins = select_bin_positions(frame)

    # 설정 창 닫기
    close_setup_window()

    if bins is None:
        print("  ❌ Bin 위치 선택 실패")
        return roi, None

    # Bin 위치 요약
    print("\n  [Bin 위치 요약]")
    for cat, (bx, by) in bins.items():
        tid = CATEGORIES[cat]
        print(f"    {cat} (id={tid}): ({bx}, {by})")

    return roi, bins


def test_step1(gemini, pipeline, roi):
    """Gemini Step 1: 객체 존재 확인"""
    print("\n[테스트] Step 1 — 객체 존재 확인...")
    frame = capture_snapshot(pipeline)
    roi_img = crop_to_roi(frame, roi)

    cv2.imshow("Step 1: ROI", roi_img)
    cv2.waitKey(1)

    result = check_objects_exist(gemini, roi_img)
    print(f"  결과: {'쓰레기 있음 ✓' if result else '비어있음 ✗'}")
    cv2.destroyAllWindows()
    cv2.waitKey(100)
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

    cv2.imshow("Step 2: Target", display)
    cv2.waitKey(1)
    print("  → 아무 키나 누르면 닫힘")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.waitKey(100)
    return target


def test_step3(gemini, roi_img, target):
    """Gemini Step 3: 카테고리 분류"""
    print("\n[테스트] Step 3 — 카테고리 분류...")
    bbox_img = crop_to_bbox(roi_img, target["bbox"])

    cv2.imshow("Step 3: Cropped", bbox_img)
    cv2.waitKey(1)

    type_id = classify_object(gemini, bbox_img)
    cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
    print(f"  결과: {cat_name} (type_id={type_id})")
    cv2.destroyAllWindows()
    cv2.waitKey(100)
    return type_id


def test_full_cycle(gemini, pipeline, roi, bins, T):
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

    # 좌표 변환 (placeholder)
    tx, ty = uv_to_robot_coords(target["center"], roi, T)

    # bbox 짧은 변 (픽셀, ROI 기준)
    ymin, xmin, ymax, xmax = target["bbox"]
    roi_h, roi_w = roi_img.shape[:2]
    bbox_w = (xmax - xmin) / GEMINI_COORD_RANGE * roi_w
    bbox_h = (ymax - ymin) / GEMINI_COORD_RANGE * roi_h
    short_side_px = int(round(min(bbox_w, bbox_h)))

    # Bin 위치
    cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
    if bins:
        bx, by = bins.get(cat_name, bins["unknown"])
    else:
        bx, by = 0.0, 0.0

    # ── Output (7개) ──────────────────────────────
    output = [type_id, tx, ty, short_side_px, target["angle"], bx, by]

    print(f"\n  📦 Output: {output}")
    print(f"     분류:  {cat_name} (type_id={type_id})")
    print(f"     좌표:  tx={tx:.2f}, ty={ty:.2f}")
    print(f"     짧은변(px): {short_side_px}")
    print(f"     각도:  {target['angle']}°")
    print(f"     쓰레기통: ({bx}, {by})")
    return output


def test_full_loop(gemini, pipeline, roi, bins, T):
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

        result = test_full_cycle(gemini, pipeline, roi, bins, T)
        if result is None:
            print("\n✅ 루프 종료!")
            break

        print("\n  [다음 사이클] 아무 키 = 계속 / q = 종료")
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        cv2.waitKey(100)
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
    T = load_transform_matrix()

    # 상태 저장
    roi = None
    bins = None

    while True:
        print_menu()
        if roi or bins:
            print_setup_status(roi, bins)
        choice = input("선택: ").strip().lower().rstrip(").")

        if choice == "1":
            test_camera(pipeline)

        elif choice == "2":
            roi, bins = test_setup(pipeline)

        elif choice == "3":
            if roi is None:
                print("⚠️  초기 설정을 먼저 하세요 (메뉴 2)")
                continue
            test_step1(gemini, pipeline, roi)

        elif choice == "4":
            if roi is None:
                print("⚠️  초기 설정을 먼저 하세요 (메뉴 2)")
                continue
            frame = capture_snapshot(pipeline)
            roi_img = crop_to_roi(frame, roi)
            test_step2(gemini, roi_img)

        elif choice == "5":
            if roi is None:
                print("⚠️  초기 설정을 먼저 하세요 (메뉴 2)")
                continue
            frame = capture_snapshot(pipeline)
            roi_img = crop_to_roi(frame, roi)
            target = select_target_object(gemini, roi_img)
            if target:
                test_step3(gemini, roi_img, target)

        elif choice == "6":
            if roi is None:
                print("⚠️  초기 설정을 먼저 하세요 (메뉴 2)")
                continue
            test_full_cycle(gemini, pipeline, roi, bins, T)

        elif choice == "7":
            if roi is None:
                print("⚠️  초기 설정을 먼저 하세요 (메뉴 2)")
                continue
            test_full_loop(gemini, pipeline, roi, bins, T)

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