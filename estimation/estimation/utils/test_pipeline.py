"""
R.O.S.T - 통합 테스트 (test_pipeline.py)
노드 없이 기능 함수들만 순서대로 테스트한다.

실행 방법:
  1) .env 파일에 API 키 넣기:  GEMINI_API_KEY=your_key_here
  2) pip install google-genai pyrealsense2 python-dotenv opencv-python
  3) python test_pipeline.py

Output: [type_id, tx, ty, short_side_px, t_angle, bx, by]
  ※ tx, ty 단위는 mm
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
from config import (
    CATEGORIES, GEMINI_COORD_RANGE, PREFER_GRASP_PTS_SHORT_SIDE,
    GRIPPER_ANGLE_USE_GRASP_NORMAL, GRIPPER_ANGLE_OFFSET_DEG_CW,
    USE_COMPLEMENTARY_GRIPPER_ANGLE,
    USE_OBJECT_CONTOUR_GEOMETRY,
)
from camera_capture import (init_camera, stop_camera,
                            capture_snapshot, crop_to_roi, crop_to_bbox)
from setup_functions import select_roi, select_bin_positions, close_setup_window
from gemini_functions_v2 import (init_gemini_client, check_objects_exist,
                              select_target_object, classify_object)
from calibration import load_transform_matrix, uv_to_robot_coords, pixel_to_robot


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
    print("  8) 수동 UV 클릭          — 중심/4코너 좌표 추출")
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


def manual_uv_click(pipeline, roi):
    """수동으로 중심/4코너를 클릭해 UV 좌표를 얻는다."""
    print("\n[수동] 중심 + 4코너를 클릭하세요 (TL → TR → BR → BL)")
    frame = capture_snapshot(pipeline)
    if roi is not None:
        view = crop_to_roi(frame, roi)
        roi_offset = (roi[0], roi[1])
    else:
        view = frame
        roi_offset = (0, 0)

    clicks = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            print(f"  클릭 {len(clicks)}: ({x},{y})")

    win = "Manual UV Click"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        display = view.copy()
        for i, (x, y) in enumerate(clicks):
            cv2.circle(display, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(display, str(i + 1), (x + 6, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow(win, display)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('r'):
            clicks.clear()
            print("  리셋")
        if key == ord('q'):
            break
        if len(clicks) >= 5:
            break

    cv2.destroyAllWindows()
    cv2.waitKey(100)

    if len(clicks) < 5:
        print("  ❌ 클릭 부족: 중심 + 4코너 필요")
        return None

    # 1번 = 중심, 2~5 = TL,TR,BR,BL
    center_px = clicks[0]
    corners_px = clicks[1:5]

    # ROI offset 적용 → 전체 이미지 기준
    center_full = (center_px[0] + roi_offset[0], center_px[1] + roi_offset[1])
    corners_full = [(x + roi_offset[0], y + roi_offset[1]) for x, y in corners_px]

    h, w = frame.shape[:2]
    def norm(pt):
        return [int(round(pt[1] / h * 1000)), int(round(pt[0] / w * 1000))]
    center_norm = norm(center_full)
    corners_norm = [[int(round(x / w * 1000)), int(round(y / h * 1000))] for x, y in corners_full]

    print("\n[결과] 전체 이미지 기준")
    print(f"  center px: {center_full}")
    print(f"  center norm(y,x): {center_norm}")
    print(f"  corners px: {corners_full}")
    print(f"  corners norm(x,y): {corners_norm}")
    return {
        "center_px": center_full,
        "center_norm": center_norm,
        "corners_px": corners_full,
        "corners_norm": corners_norm,
    }


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

    # OBB 시각화
    h, w = roi_img.shape[:2]
    px = lambda val, size: int(val / 1000 * size)
    corners = target["corners"]  # TL, TR, BR, BL (x,y)
    pts = np.array([(px(x, w), px(y, h)) for x, y in corners], dtype=np.int32)
    pts = pts.reshape((-1, 1, 2))
    cy, cx = target["center"]
    center_px = (px(cx, w), px(cy, h))

    display = roi_img.copy()
    cv2.polylines(display, [pts], True, (0, 255, 0), 2)
    cv2.circle(display, center_px, 5, (0, 0, 255), -1)
    # grasp line
    if "grasp_pts" in target:
        (gx1, gy1), (gx2, gy2) = target["grasp_pts"]
        g1 = (px(gx1, w), px(gy1, h))
        g2 = (px(gx2, w), px(gy2, h))
        cv2.line(display, g1, g2, (255, 0, 0), 2)
    p0 = tuple(pts[0][0])
    cv2.putText(display, f"{target['label']} ({target['angle']:.1f}deg)",
                (p0[0], p0[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print(f"  corners(norm): {target['corners']}")
    print(f"  center(norm): {target['center']}  angle(deg): {target['angle']:.1f}")
    debug_path = "/tmp/rost_step2_debug.png"
    cv2.imwrite(debug_path, display)
    print(f"  [DEBUG] OBB overlay saved: {debug_path}")

    # raw corners swap 디버그 이미지
    raw = target.get("raw_corners")
    if raw:
        raw_pts = np.array([(px(x, w), px(y, h)) for x, y in raw], dtype=np.int32).reshape((-1, 1, 2))
        raw_swap_pts = np.array([(px(y, w), px(x, h)) for x, y in raw], dtype=np.int32).reshape((-1, 1, 2))
        dbg_raw = roi_img.copy()
        cv2.polylines(dbg_raw, [raw_pts], True, (255, 0, 0), 2)
        cv2.imwrite("/tmp/rost_step2_debug_raw.png", dbg_raw)
        dbg_swap = roi_img.copy()
        cv2.polylines(dbg_swap, [raw_swap_pts], True, (0, 0, 255), 2)
        cv2.imwrite("/tmp/rost_step2_debug_raw_swap.png", dbg_swap)
        print("  [DEBUG] raw corners overlays saved:")
        print("          /tmp/rost_step2_debug_raw.png (as-is)")
        print("          /tmp/rost_step2_debug_raw_swap.png (x/y swapped)")

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

    type_id = classify_object(gemini, bbox_img, label_hint=target.get("label", ""))
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

    # 물체 윤곽 기반으로 짧은변/각도 산출 (사진 픽셀 기준)
    def estimate_geom_from_contour(roi_img, bbox_norm, roi_rect, Tmat):
        if not (isinstance(bbox_norm, (list, tuple)) and len(bbox_norm) == 4):
            return None
        roi_h, roi_w = roi_img.shape[:2]
        ymin, xmin, ymax, xmax = [float(v) for v in bbox_norm]
        ymin, ymax = min(ymin, ymax), max(ymin, ymax)
        xmin, xmax = min(xmin, xmax), max(xmin, xmax)
        x1 = int(round(xmin / GEMINI_COORD_RANGE * roi_w))
        y1 = int(round(ymin / GEMINI_COORD_RANGE * roi_h))
        x2 = int(round(xmax / GEMINI_COORD_RANGE * roi_w))
        y2 = int(round(ymax / GEMINI_COORD_RANGE * roi_h))
        mx = int((x2 - x1) * 0.12)
        my = int((y2 - y1) * 0.12)
        x1, y1 = max(0, x1 - mx), max(0, y1 - my)
        x2, y2 = min(roi_w, x2 + mx), min(roi_h, y2 + my)
        if x2 <= x1 or y2 <= y1:
            return None

        crop = roi_img[y1:y2, x1:x2]
        if crop.size == 0:
            return None
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_inv = cv2.bitwise_not(th)

        def pick_rect(mask):
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best = None
            best_area = 0.0
            H, W = mask.shape[:2]
            for c in cnts:
                area = cv2.contourArea(c)
                if area < 200:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                if w <= 2 or h <= 2:
                    continue
                # 배경 전체를 집는 경우 배제
                if w > 0.97 * W and h > 0.97 * H:
                    continue
                if area > best_area:
                    best_area = area
                    best = cv2.minAreaRect(c)
            return best, best_area

        rect_a, area_a = pick_rect(th)
        rect_b, area_b = pick_rect(th_inv)
        rect = rect_a if area_a >= area_b else rect_b
        if rect is None:
            return None

        box = cv2.boxPoints(rect)  # 4x2 float
        box[:, 0] += x1
        box[:, 1] += y1
        box = box.astype(np.float32)

        def dist(p, q):
            return float(np.hypot(p[0] - q[0], p[1] - q[1]))

        edges = []
        for i in range(4):
            j = (i + 1) % 4
            edges.append((dist(box[i], box[j]), i, j))
        dmin, imin, jmin = min(edges, key=lambda x: x[0])
        p1, p2 = box[imin], box[jmin]

        roi_x, roi_y, _, _ = roi_rect
        u1, v1 = int(round(roi_x + p1[0])), int(round(roi_y + p1[1]))
        u2, v2 = int(round(roi_x + p2[0])), int(round(roi_y + p2[1]))
        tx1, ty1 = pixel_to_robot(u1, v1, Tmat)
        tx2, ty2 = pixel_to_robot(u2, v2, Tmat)
        vx, vy = (tx2 - tx1), (ty2 - ty1)
        short_mm = int(round((vx ** 2 + vy ** 2) ** 0.5))

        ang = (np.degrees(np.arctan2(vy, vx)) + 180.0) % 180.0
        # contour 짧은변 기준 yaw는 짧은변 직교방향이 그리퍼 접근방향인 경우가 많음
        if GRIPPER_ANGLE_USE_GRASP_NORMAL:
            ang = (ang + 90.0) % 180.0
        if USE_COMPLEMENTARY_GRIPPER_ANGLE:
            ang = (180.0 - ang) % 180.0
        ang = (ang + GRIPPER_ANGLE_OFFSET_DEG_CW) % 180.0

        short_px = int(round(dmin))
        return short_mm, float(ang), short_px

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
        short_side_src = "target_short_side_px"
        seg_norm = (grasp_pts[0], grasp_pts[1])
        seg_is_grasp = True
    else:
        # fallback: corners / grasp_pts 기반
        corners = target["corners"]
        pts = [(px(x, roi_w), px(y, roi_h)) for x, y in corners]

        def dist(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

        edge_info = []
        for i in range(4):
            j = (i + 1) % 4
            d = dist(pts[i], pts[j])
            edge_info.append((d, i, j))
        dmin, imin, jmin = min(edge_info, key=lambda x: x[0])
        short_side_from_corners = dmin
        short_side_px = int(round(short_side_from_corners))
        short_side_src = "corners"
        seg_norm = (corners[imin], corners[jmin])

        if PREFER_GRASP_PTS_SHORT_SIDE and isinstance(grasp_pts, list) and len(grasp_pts) == 2:
            (gx1, gy1), (gx2, gy2) = grasp_pts
            gp1 = (px(gx1, roi_w), px(gy1, roi_h))
            gp2 = (px(gx2, roi_w), px(gy2, roi_h))
            grasp_dist = dist(gp1, gp2)
            ratio = grasp_dist / max(1.0, short_side_from_corners)
            # grasp 선분이 corners 짧은변과 길이 일관될 때만 채택
            if grasp_dist >= 3.0 and 0.6 <= ratio <= 1.4:
                short_side_px = int(round(grasp_dist))
                short_side_src = "grasp_pts"
                seg_norm = (grasp_pts[0], grasp_pts[1])
                seg_is_grasp = True

    # 우선: contour 기반(사진 자체 물체 윤곽)
    contour_geom = None
    if USE_OBJECT_CONTOUR_GEOMETRY:
        contour_geom = estimate_geom_from_contour(roi_img, target.get("bbox"), roi, T)

    # fallback: 선분 기반 변환
    roi_x, roi_y, roi_w, roi_h = roi
    if contour_geom is not None:
        short_side_mm, angle_out, contour_short_px = contour_geom
        short_side_src = f"contour(raw={contour_short_px}px)"
    elif seg_norm is None:
        print("[경고] short side 선분 없음 → target angle 사용")
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

    bbox_mode = target.get("bbox_mode", "obb")

    # Bin 위치
    cat_name = [k for k, v in CATEGORIES.items() if v == type_id][0]
    if bins:
        bx, by = bins.get(cat_name, bins["unknown"])
    else:
        bx, by = 0.0, 0.0

    # ── Output (7개) ──────────────────────────────
    output = [type_id, tx, ty, short_side_mm, float(angle_out), bx, by]

    print(f"\n  📦 Output: {output}")
    print(f"     분류:  {cat_name} (type_id={type_id})")
    print(f"     좌표:  tx={tx:.2f}, ty={ty:.2f}")
    print(f"     짧은변: {short_side_mm} mm (raw={short_side_px}px, {short_side_src}, mode={bbox_mode})")
    print(f"     각도:  {angle_out:.2f}° (robot-frame from short-side segment)")
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
    calib_path = os.path.join(os.path.dirname(__file__), "camcalib.npz")
    T = load_transform_matrix(filepath=calib_path)

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

        elif choice == "8":
            if roi is None:
                print("⚠️  초기 설정을 먼저 하세요 (메뉴 2)")
                continue
            manual_uv_click(pipeline, roi)

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
