"""
R.O.S.T - Gemini API 함수 (gemini_functions_v2.py)  ★ 핵심 모듈

gemini-robotics-er-1.5-preview 3단계 호출:
  Step 1: 객체 존재 확인  ("쓰레기 남아있어?")
  Step 2: 타겟 선정       ("가장 집기 쉬운 거 골라 → 위치+각도")
  Step 3: 카테고리 분류    ("이게 뭐야? → 7종 중 1개")

[반영된 개선사항 — 기존 estimation 코드에서 가져옴]
  1. _parse_json()     → 다단계 fallback 파싱
  2. _to_bytes()       → shape/dtype 방어
  3. _call_gemini()    → retry + 지수 백오프
  4. _match_category() → word-boundary 매칭 안전장치
"""

import json
import math
import re
import time
import cv2
import numpy as np
from google import genai
from google.genai import types
from config import (
    GEMINI_API_KEY, GEMINI_MODEL, GEMINI_TEMPERATURE,
    GEMINI_THINKING_BUDGET_SPATIAL, GEMINI_THINKING_BUDGET_CLASSIFY,
    CATEGORY_LIST, CATEGORIES,
    PREFER_GRASP_PTS_ANGLE, GRIPPER_ANGLE_OFFSET_DEG_CW,
    GRIPPER_ANGLE_USE_GRASP_NORMAL, USE_DIRECT_TARGET_PICK,
)

# OBB corners 해석 강제 스왑
FORCE_SWAP_CORNERS = True
# 로봇 기준 x축이 이미지 세로(y+)와 일치하면 True
ROBOT_X_AXIS_IS_IMAGE_VERTICAL = True


# ── 공통 유틸 ─────────────────────────────────────────

def init_gemini_client():
    """Gemini 클라이언트 생성"""
    assert GEMINI_API_KEY != "YOUR_API_KEY_HERE", \
        "config.py에서 GEMINI_API_KEY를 설정하세요"
    client = genai.Client(api_key=GEMINI_API_KEY)
    print(f"[Gemini] 초기화 완료 ({GEMINI_MODEL})")
    return client


def _to_bytes(image: np.ndarray) -> bytes:
    """
    OpenCV BGR → JPEG bytes.
    [개선 #2] shape 검증 + dtype 강제 변환 + quality clamp
    """
    # [안전장치] None / 빈 배열
    if image is None or not hasattr(image, "shape"):
        raise ValueError("이미지가 None이거나 numpy 배열이 아닙니다")
    # [안전장치] 3채널 BGR 확인
    if len(image.shape) != 3 or image.shape[2] != 3:
        raise ValueError(f"BGR 3채널 필요. 현재 shape: {image.shape}")
    # [안전장치] dtype 강제 변환
    if image.dtype != np.uint8:
        image = image.astype(np.uint8, copy=False)
    # [수정 포인트] JPEG 품질을 바꾸려면 여기만
    quality = max(1, min(100, 90))
    ok, buf = cv2.imencode('.jpg', image,
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG 인코딩 실패")
    return buf.tobytes()


def _parse_json(text: str):
    """
    Gemini 응답 → JSON 파싱.
    [개선 #1] 다단계 fallback: 코드펜스 제거 → 그대로 시도 → {} 추출 → [] 추출
    """
    if not text or not text.strip():
        raise json.JSONDecodeError("빈 응답", text or "", 0)

    cleaned = text.strip()

    # 1단계: 코드펜스 제거
    fence = re.search(r"```(?:json)?\s*(.*?)```", cleaned, re.DOTALL)
    if fence:
        cleaned = fence.group(1).strip()

    # 2단계: 그대로 파싱 시도
    try:
        return json.loads(cleaned)
    except json.JSONDecodeError:
        pass

    # 3단계: { } 영역 추출 시도
    b1, b2 = cleaned.find("{"), cleaned.rfind("}")
    if b1 != -1 and b2 > b1:
        try:
            return json.loads(cleaned[b1:b2 + 1])
        except json.JSONDecodeError:
            pass

    # 4단계: [ ] 영역 추출 시도
    a1, a2 = cleaned.find("["), cleaned.rfind("]")
    if a1 != -1 and a2 > a1:
        try:
            return json.loads(cleaned[a1:a2 + 1])
        except json.JSONDecodeError:
            pass

    raise json.JSONDecodeError(f"JSON 파싱 불가 (길이={len(text)})", text, 0)


# [수정 포인트] 재시도 횟수/대기시간 바꾸려면 여기만
_MAX_RETRIES = 2
_BACKOFF_SEC = 0.5


def _call_gemini(client, image: np.ndarray, prompt: str,
                 temperature: float, thinking_budget: int):
    """
    Gemini API 호출 래퍼.
    [개선 #3] retry + 지수 백오프 + 빈 응답 재시도
    """
    img_bytes = _to_bytes(image)
    last_err = None

    for attempt in range(_MAX_RETRIES + 1):
        try:
            resp = client.models.generate_content(
                model=GEMINI_MODEL,
                contents=[
                    types.Part.from_bytes(data=img_bytes, mime_type='image/jpeg'),
                    prompt
                ],
                config=types.GenerateContentConfig(
                    temperature=temperature,
                    thinking_config=types.ThinkingConfig(
                        thinking_budget=thinking_budget)))
            # [안전장치] 빈 응답 방어
            if not (resp.text or "").strip():
                raise RuntimeError("빈 응답")
            return resp

        except Exception as e:
            last_err = e
            if attempt < _MAX_RETRIES:
                wait = _BACKOFF_SEC * (2 ** attempt)
                print(f"[Gemini] 재시도 {attempt+1}/{_MAX_RETRIES} "
                      f"({wait:.1f}s 대기): {e}")
                time.sleep(wait)

    raise RuntimeError(f"Gemini {_MAX_RETRIES+1}회 실패: {last_err}")


def _match_category(label: str) -> str:
    """
    Gemini 라벨 → 카테고리 매칭.
    [개선 #4] word-boundary 매칭으로 "candy"→"can" 같은 오분류 차단.
              "plastic bottle" → "plastic" ✓
              "candy wrapper"  → "can" ✗ (경계 불일치)
    """
    norm = (label or "").strip().lower()
    if not norm or norm == "unknown":
        return "unknown"

    # 1순위: 정확히 일치
    if norm in CATEGORIES:
        return norm

    # 2순위: 단어 단위 매칭 (부분 문자열 아님!)
    for key in CATEGORIES:
        if re.search(rf"\b{re.escape(key)}\b", norm):
            return key

    return "unknown"


# ── Step 1: 객체 존재 확인 ────────────────────────────

def check_objects_exist(client, roi_image: np.ndarray) -> bool:
    """ROI 안 쓰레기 유무 → True/False. False면 루프 종료."""
    resp = _call_gemini(client, roi_image,
        "Look at this waste collection area. "
        "Is there any waste object remaining? Answer ONLY 'yes' or 'no'.",
        temperature=0.1, thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
    result = "yes" in resp.text.strip().lower()
    print(f"[Step 1] 객체: {'있음 ✓' if result else '없음 → 종료'}")
    return result


# ── Step 2: 타겟 선정 + 위치/각도 ─────────────────────

# [수정 포인트] 프롬프트 바꾸려면 여기만
_P2 = """Look at this waste collection area. Pick the ONE object that is easiest to grab with a parallel gripper — meaning it's the most isolated and has clear edges.

Important:
- If an object has a wrapper, label, or packaging around it, that's still one single item. Draw the box around just the main object, not the packaging separately.
- If objects are stacked or overlapping, pick only the top one and fit the box tightly around it alone.
- Do NOT merge nearby objects into one box. Each object is separate.

Return ONLY this JSON, nothing else:
{"corners": [[x1,y1],[x2,y2],[x3,y3],[x4,y4]], "grasp_pts": [[xg1,yg1],[xg2,yg2]], "label": "<short description>"}

Rules for the JSON values:
- All coordinates are normalized 0 to 1000.
- "corners" must be 4 points in this exact order: Top-left, Top-right, Bottom-right, Bottom-left.
- Each point is [x, y] (x first, y second).
- The coordinate system is THIS IMAGE ONLY (top-left = [0,0], bottom-right = [1000,1000]).
- Do NOT use any original full-frame coordinates; use the image you see here.
- "grasp_pts" are the midpoints of the two opposite faces that the parallel gripper should squeeze.
- These two points must be on the object surface (not outside), and define the true object yaw.
- The box must be TIGHT: do NOT include background or nearby objects.
- If your box includes any other object, redo it tighter around the chosen object.
- "label" is a brief description like "crushed plastic cup" or "tomato can".
- Every field is required. Always include corners, grasp_pts, and label.
- CRITICAL: All 4 corner points MUST be different coordinates. 
  Do NOT repeat the same point. If the object is axis-aligned (like a box), 
  you still need 4 distinct corners:
  Example for a horizontal box: [[100,200],[400,200],[400,400],[100,400]]
  WRONG: [[100,200],[100,200],[400,400],[400,400]] ← only 2 unique points!
- Even if the object is perfectly rectangular and aligned with the image axes, 
  always return 4 unique corners in TL, TR, BR, BL order."""

_P2_FALLBACK = """Pick ONE object that is easiest to grab. If multiple objects, choose the most isolated with clear edges.
Return ONLY this JSON:
{"corners": [[x1,y1],[x2,y2],[x3,y3],[x4,y4]], "grasp_pts": [[xg1,yg1],[xg2,yg2]], "label": "<short>"}
Rules:
- Coordinates are normalized 0..1000 (top-left = 0,0).
- corners order: TL, TR, BR, BL. Each point is [x,y].
- grasp_pts are midpoints of the two faces the gripper should squeeze.
- Always return JSON; never say you can't.
- All 4 corners must be different. Never repeat a point."""

_P2_TIGHTEN = """Your last box was too large. Redo it.
Pick ONE object and draw a TIGHT rotated box around ONLY that object.
Do NOT include any other object or background.
Return ONLY this JSON:
{"corners": [[x1,y1],[x2,y2],[x3,y3],[x4,y4]], "grasp_pts": [[xg1,yg1],[xg2,yg2]], "label": "<short>"}
Rules:
- Coordinates are normalized 0..1000 (top-left = 0,0).
- corners order: TL, TR, BR, BL. Each point is [x,y].
- grasp_pts are midpoints of the two faces the gripper should squeeze.
- If any other object would be inside the box, it is incorrect.
- All 4 corners must be different coordinates. Never repeat a point."""

_P2_AABB = """If rotated corners are hard, return an axis-aligned box instead.
Return ONLY this JSON:
{"box_2d": [ymin, xmin, ymax, xmax], "label": "<short>"}
Rules:
- Coordinates are normalized 0..1000 (top-left = 0,0).
- Box must be tight and include only the chosen object."""

_P2_DIRECT = """Pick ONE object that is easiest to grab with a parallel gripper.
Return ONLY this JSON:
{"center":[cy,cx], "grasp_pts":[[x1,y1],[x2,y2]], "label":"<short>"}
Rules:
- Coordinates are normalized 0..1000.
- center is [y,x].
- grasp_pts are [x,y] and must be on two opposite faces that the gripper should squeeze.
- Do not include any extra text."""


def select_target_object(client, roi_image: np.ndarray) -> dict | None:
    """가장 집기 쉬운 객체 1개 → OBB corners 기반 bbox/center/angle. 실패→None."""
    def _parse_step2_direct(resp):
        r = _parse_json(resp.text)
        for k in ("grasp_pts",):
            assert k in r, f"'{k}' 없음"

        grasp_pts = r["grasp_pts"]
        if not (isinstance(grasp_pts, list) and len(grasp_pts) == 2):
            print(f"[에러] direct grasp_pts 형식 이상: {grasp_pts}")
            return None

        safe_g = []
        for p in grasp_pts:
            if not (isinstance(p, list) and len(p) == 2):
                print(f"[에러] direct grasp point 형식 이상: {p}")
                return None
            try:
                x, y = int(p[0]), int(p[1])
            except (ValueError, TypeError):
                print(f"[에러] direct grasp point 값이 숫자가 아님: {p}")
                return None
            if not (0 <= x <= 1000 and 0 <= y <= 1000):
                print(f"[에러] direct grasp point 범위초과: {p}")
                return None
            safe_g.append([x, y])

        h, w = roi_image.shape[:2]
        to_px = lambda p: (int(round(p[0] / 1000 * w)), int(round(p[1] / 1000 * h)))
        g1_px, g2_px = to_px(safe_g[0]), to_px(safe_g[1])
        gvx, gvy = g2_px[0] - g1_px[0], g2_px[1] - g1_px[1]
        glen = (gvx ** 2 + gvy ** 2) ** 0.5
        if glen < 3.0:
            print(f"[에러] direct grasp line 너무 짧음: {glen:.1f}px")
            return None

        # center는 응답값 우선, 없으면 grasp midpoint 사용
        if isinstance(r.get("center"), list) and len(r["center"]) == 2:
            try:
                cy, cx = int(r["center"][0]), int(r["center"][1])
            except (ValueError, TypeError):
                cy, cx = None, None
            if cy is None or cx is None or not (0 <= cy <= 1000 and 0 <= cx <= 1000):
                cy, cx = None, None
        else:
            cy, cx = None, None
        if cy is None or cx is None:
            cx = int(round((safe_g[0][0] + safe_g[1][0]) / 2))
            cy = int(round((safe_g[0][1] + safe_g[1][1]) / 2))

        center = [cy, cx]

        angle_img = (math.degrees(math.atan2(gvy, gvx)) + 180) % 180
        if GRIPPER_ANGLE_USE_GRASP_NORMAL:
            angle_img = (angle_img + 90.0) % 180.0
        if ROBOT_X_AXIS_IS_IMAGE_VERTICAL:
            angle = (angle_img - 90 + 180) % 180
        else:
            angle = angle_img
        angle = (angle + GRIPPER_ANGLE_OFFSET_DEG_CW) % 180

        short_side_px = int(round(glen))
        pad = max(12, int(round(short_side_px * 0.6)))
        xmin = max(0, min(safe_g[0][0], safe_g[1][0]) - pad)
        xmax = min(1000, max(safe_g[0][0], safe_g[1][0]) + pad)
        ymin = max(0, min(safe_g[0][1], safe_g[1][1]) - pad)
        ymax = min(1000, max(safe_g[0][1], safe_g[1][1]) + pad)
        bbox = [ymin, xmin, ymax, xmax]
        corners = [[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]]

        out = {
            "center": center,
            "grasp_pts": safe_g,
            "bbox": bbox,
            "corners": corners,
            "short_side_px": short_side_px,
            "angle": float(angle),
            "bbox_mode": "grasp",
            "label": r.get("label", "?"),
            "raw_corners": corners,
        }
        print(f"[Step 2][direct] '{out['label']}' center={out['center']} "
              f"angle={out['angle']:.1f}° short={short_side_px}px")
        return out

    def _parse_step2(resp):
        r = _parse_json(resp.text)

        for k in ("corners", "grasp_pts"):                                 # [안전장치] 필수키
            assert k in r, f"'{k}' 없음"

        corners = r["corners"]
        if not (isinstance(corners, list) and len(corners) == 4):
            print(f"[에러] corners 형식 이상: {corners}")
            return None

        # ── corners 값 방어 ───────────────────────────────
        safe_corners = []
        for p in corners:
            if not (isinstance(p, list) and len(p) == 2):
                print(f"[에러] corner 형식 이상: {p}")
                return None
            try:
                x, y = int(p[0]), int(p[1])
            except (ValueError, TypeError):
                print(f"[에러] corner 값이 숫자가 아님: {p}")
                return None
            if not (0 <= x <= 1000 and 0 <= y <= 1000):
                print(f"[에러] corner 범위초과: {p}")
                return None
            safe_corners.append([x, y])

        # ── grasp_pts 값 방어 (corners와 동일 좌표계 유지) ───
        grasp_pts = r["grasp_pts"]
        if not (isinstance(grasp_pts, list) and len(grasp_pts) == 2):
            print(f"[에러] grasp_pts 형식 이상: {grasp_pts}")
            return None
        safe_grasp_pts = []
        for p in grasp_pts:
            if not (isinstance(p, list) and len(p) == 2):
                print(f"[에러] grasp point 형식 이상: {p}")
                return None
            try:
                x, y = int(p[0]), int(p[1])
            except (ValueError, TypeError):
                print(f"[에러] grasp point 값이 숫자가 아님: {p}")
                return None
            if not (0 <= x <= 1000 and 0 <= y <= 1000):
                print(f"[에러] grasp point 범위초과: {p}")
                return None
            safe_grasp_pts.append([x, y])

        # ── 해석 선택 (x,y vs y,x) ───────────────────────
        h, w = roi_image.shape[:2]
        def to_px(points, swap=False):
            if not swap:
                return [(int(round(x / 1000 * w)), int(round(y / 1000 * h))) for x, y in points]
            return [(int(round(y / 1000 * w)), int(round(x / 1000 * h))) for x, y in points]

        def order_points(points):
            cx = sum(p[0] for p in points) / 4.0
            cy = sum(p[1] for p in points) / 4.0
            ang = [math.atan2(p[1] - cy, p[0] - cx) for p in points]
            pts = [p for _, p in sorted(zip(ang, points))]
            tl_idx = min(range(4), key=lambda i: (pts[i][1], pts[i][0]))
            ordered = pts[tl_idx:] + pts[:tl_idx]
            return ordered

        def order_with_indices(points):
            cx = sum(p[0] for p in points) / 4.0
            cy = sum(p[1] for p in points) / 4.0
            ang = [math.atan2(p[1] - cy, p[0] - cx) for p in points]
            idx_sorted = [i for _, i in sorted(zip(ang, range(4)))]
            tl_idx = min(range(4), key=lambda i: (points[idx_sorted[i]][1], points[idx_sorted[i]][0]))
            ordered_idx = idx_sorted[tl_idx:] + idx_sorted[:tl_idx]
            ordered = [points[i] for i in ordered_idx]
            return ordered, ordered_idx

        def score(points):
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            bbox_area = max(1.0, (xmax - xmin) * (ymax - ymin))
            area = 0.0
            for i in range(4):
                x0, y0 = points[i]
                x1, y1 = points[(i + 1) % 4]
                area += (x0 * y1 - x1 * y0)
            area = abs(area) * 0.5
            ratio = area / bbox_area
            return (ratio, area)

        px_normal = order_points(to_px(safe_corners, swap=False))
        px_swap = order_points(to_px(safe_corners, swap=True))
        s_normal = score(px_normal)
        s_swap = score(px_swap)

        use_swap = FORCE_SWAP_CORNERS or (s_swap > s_normal)
        if use_swap:
            safe_corners = [[y, x] for x, y in safe_corners]  # swap to true [x,y]
            safe_grasp_pts = [[y, x] for x, y in safe_grasp_pts]
            print("[보정] corners 해석을 (y,x)→(x,y)로 스왑")
        else:
            print(f"[INFO] corners 해석 유지 (score normal={s_normal}, swap={s_swap})")
        # ── order 보정: centroid 기준 각도 정렬 → TL 시작 ──
        # safe_corners는 normalized (x,y)로 유지
        safe_corners, order_idx = order_with_indices(safe_corners)
        px_ordered = order_points(to_px(safe_corners, swap=False))

        # ── 면적/중복 검사 (너무 작거나 일직선이면 실패) ─────
        uniq = {(p[0], p[1]) for p in safe_corners}
        if len(uniq) < 4:
            print(f"[에러] corner 중복: {safe_corners}")
            return None
        area = 0.0
        for i in range(4):
            x1, y1 = safe_corners[i]
            x2, y2 = safe_corners[(i + 1) % 4]
            area += (x1 * y2 - x2 * y1)
        if abs(area) < 1.0:
            print(f"[에러] corner 면적이 너무 작음: {safe_corners}")
            return None

        # ── 다각형 중심(centroid) 계산 (pixel 기준) ───────
        # shoelace 기반 centroid (순서가 시계/반시계여도 OK)
        area2 = 0.0
        cx_num = 0.0
        cy_num = 0.0
        for i in range(4):
            x0, y0 = px_ordered[i]
            x1, y1 = px_ordered[(i + 1) % 4]
            cross = x0 * y1 - x1 * y0
            area2 += cross
            cx_num += (x0 + x1) * cross
            cy_num += (y0 + y1) * cross
        if abs(area2) < 1e-6:
            xs = [p[0] for p in px_ordered]
            ys = [p[1] for p in px_ordered]
            cx_px = sum(xs) / 4.0
            cy_px = sum(ys) / 4.0
        else:
            cx_px = cx_num / (3 * area2)
            cy_px = cy_num / (3 * area2)

        cx_norm = int(round(cx_px / w * 1000))
        cy_norm = int(round(cy_px / h * 1000))
        center = [cy_norm, cx_norm]

        # ── axis-aligned bbox 계산 (Step3 크롭용) ───────
        xs = [p[0] for p in safe_corners]
        ys = [p[1] for p in safe_corners]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        bbox = [ymin, xmin, ymax, xmax]

        # ── OBB 유효성 체크 (너무 얇은 경우) ──────────────
        def _dist(a, b):
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        edges_px = [_dist(px_ordered[i], px_ordered[(i + 1) % 4]) for i in range(4)]
        min_edge = min(edges_px)
        if min_edge < 10:
            print(f"[에러] OBB 너무 얇음 (min_edge={min_edge:.1f}px)")
            return None

        # ── corners 기반 angle 계산 (물체 자체 각도) ──────
        # corners: TL, TR, BR, BL (pixel)
        tl, tr, br, bl = px_ordered
        v1 = (tr[0] - tl[0], tr[1] - tl[1])  # 상단 변
        v2 = (br[0] - tr[0], br[1] - tr[1])  # 우측 변
        len1 = (v1[0] ** 2 + v1[1] ** 2) ** 0.5
        len2 = (v2[0] ** 2 + v2[1] ** 2) ** 0.5
        vx, vy = v1 if len1 >= len2 else v2
        angle_img_corner = (math.degrees(math.atan2(vy, vx)) + 180) % 180
        # 로봇 기준으로 각도 변환 (이미지 세로가 로봇 x축인 경우)
        if ROBOT_X_AXIS_IS_IMAGE_VERTICAL:
            angle_corner = (angle_img_corner - 90 + 180) % 180
        else:
            angle_corner = angle_img_corner

        # ── grasp_pts 기반 angle (가능하면 이쪽이 집기 방향과 더 직접적) ──
        angle = angle_corner
        angle_source = "corners"
        if len(safe_grasp_pts) == 2:
            gpx = to_px(safe_grasp_pts, swap=False)
            gvx = gpx[1][0] - gpx[0][0]
            gvy = gpx[1][1] - gpx[0][1]
            glen = (gvx ** 2 + gvy ** 2) ** 0.5
            if glen >= 3.0:
                angle_img_grasp = (math.degrees(math.atan2(gvy, gvx)) + 180) % 180
                if ROBOT_X_AXIS_IS_IMAGE_VERTICAL:
                    angle_grasp = (angle_img_grasp - 90 + 180) % 180
                else:
                    angle_grasp = angle_img_grasp
                if PREFER_GRASP_PTS_ANGLE:
                    angle = angle_grasp
                    angle_source = "grasp_pts"

        # 현장 미세 보정 (시계방향 +deg)
        angle = (angle + GRIPPER_ANGLE_OFFSET_DEG_CW) % 180

        out = {"corners": safe_corners, "grasp_pts": safe_grasp_pts,
               "bbox": bbox, "center": center,
               "angle": float(angle), "bbox_mode": "obb", "label": r.get("label", "?"),
               "raw_corners": corners}
        print(f"[Step 2] '{out['label']}' center={out['center']} "
              f"angle={out['angle']:.1f}° ({angle_source})")
        return out

    def _parse_aabb(resp):
        r = _parse_json(resp.text)
        for k in ("box_2d",):
            assert k in r, f"'{k}' 없음"
        box = r["box_2d"]
        if not (isinstance(box, list) and len(box) == 4):
            raise AssertionError(f"box_2d 형식 이상: {box}")
        ymin, xmin, ymax, xmax = [int(v) for v in box]
        ymin, ymax = min(ymin, ymax), max(ymin, ymax)
        xmin, xmax = min(xmin, xmax), max(xmin, xmax)
        ymin = max(0, min(1000, ymin))
        ymax = max(0, min(1000, ymax))
        xmin = max(0, min(1000, xmin))
        xmax = max(0, min(1000, xmax))
        corners = [[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]]
        center = [int(round((ymin + ymax) / 2)), int(round((xmin + xmax) / 2))]
        out = {"corners": corners, "bbox": [ymin, xmin, ymax, xmax], "center": center,
               "angle": 0.0, "bbox_mode": "aabb", "label": r.get("label", "?"),
               "raw_corners": corners}
        print(f"[Step 2] '{out['label']}' AABB fallback")
        return out
    def _area_ratio(corners):
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        bbox_area = max(1.0, (xmax - xmin) * (ymax - ymin))
        # polygon area (shoelace)
        area = 0.0
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            area += x0 * y1 - x1 * y0
        area = abs(area) * 0.5
        return area / bbox_area
    try:
        # direct 모드 (선택적으로만 사용)
        if USE_DIRECT_TARGET_PICK:
            try:
                resp_d = _call_gemini(client, roi_image, _P2_DIRECT,
                                      temperature=GEMINI_TEMPERATURE,
                                      thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
                out_d = _parse_step2_direct(resp_d)
                if out_d is not None:
                    return out_d
            except Exception as e_d:
                print(f"[Step2] direct 실패 → legacy fallback: {e_d}")

        resp = _call_gemini(client, roi_image, _P2,
                            temperature=GEMINI_TEMPERATURE,
                            thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
        out = _parse_step2(resp)
        if out is None:
            # fallback to axis-aligned bbox
            print("[Step2] OBB 실패 → AABB fallback")
            try:
                resp_a = _call_gemini(client, roi_image, _P2_AABB,
                                      temperature=0.1,
                                      thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
                return _parse_aabb(resp_a)
            except (json.JSONDecodeError, KeyError, AssertionError) as e:
                print(f"[에러] Step2 AABB 실패: {e}\n  원본: {resp_a.text}")
                return None
        # 크기 과대(ROI 대비)면 타이트닝 프롬프트 재시도
        ratio = _area_ratio(out["corners"])
        if ratio < 0.6:
            print(f"[Step2] 박스 타이트닝 재시도 (area ratio={ratio:.2f})")
            resp_t = _call_gemini(client, roi_image, _P2_TIGHTEN,
                                  temperature=0.1,
                                  thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
            out_t = _parse_step2(resp_t)
            if out_t is not None:
                out = out_t
            else:
                print("[Step2] 타이트닝 실패 → 기존 박스 유지")
        return out
    except (json.JSONDecodeError, KeyError, AssertionError) as e:
        print(f"[에러] Step2: {e}\n  원본: {resp.text}")
        print("[Step2] fallback 프롬프트로 재시도")
        try:
            resp2 = _call_gemini(client, roi_image, _P2_FALLBACK,
                                 temperature=0.2,
                                 thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
            return _parse_step2(resp2)
        except (json.JSONDecodeError, KeyError, AssertionError) as e2:
            print(f"[에러] Step2 fallback 실패: {e2}\n  원본: {resp2.text}")
            return None


# ── Step 3: 카테고리 분류 (증거 기반) ─────────────────
# First, try to identify WHAT this object is (e.g., a can, a cup, a bottle, a bag, a box).
# Look at the overall shape, size, and structure — not just the surface texture.
# Then, classify based on what the object actually IS, not what its surface looks like.

# [수정 포인트] 분류 프롬프트 바꾸려면 여기만
_P3 = f"""Look at this single waste object very carefully. Your job is to classify it into ONE of these categories: {', '.join(CATEGORY_LIST)}
Before classifying, imagine picking this object up with your gripper.
- Would it feel rigid and hold its shape? → plastic or can or glass
- Would it crumple flat with almost no resistance, like a thin sheet? → vinyl
- Would it feel like paper? → paper or box

A crushed plastic cup is still rigid plastic — it resists being flattened 
and springs back partially. Vinyl film just collapses flat like a deflated balloon.

For example: a tin can covered in a colorful paper label is still a "can" because the object itself is a metal container. Don't be fooled by surface wrapping.

But here's the important rule: you must PROVE your classification with visual evidence. If you can't find enough evidence, classify it as "unknown". It's much better to say "unknown" than to guess wrong.

For each category, you need to see AT LEAST 2 of these clues:

can (metal containers):
- Metallic sheen or reflective surface typical of aluminum/tin
- Cylindrical body shape (even if dented or crushed)
- Visible rim or lip at the top/bottom edge
- Pull-tab or opening mechanism
- NOTE: Even if a can has a paper label or plastic wrap around it, the BODY is metal → it's a can

plastic (rigid plastic items):
- Circular or oval rim/edge (even partial arc) suggesting a cup or bottle
- Concave structure or "inward fold" from a crushed container
- Thick, rigid-looking material with strong surface highlights (small bright reflections)
- Visible thread pattern from a bottle cap area
- NOTE: Even if crushed or crumpled, if you can tell it was originally a rigid container → it's plastic

vinyl (soft film/sheet material):
- Thin, flexible sheet with multiple translucent layers overlapping
- Handle loops or bag opening structure
- Very fine, complex, fractal-like wrinkles with no clear directional pattern
- Material appears extremely thin and crinkly
- STRICT RULE: If you do NOT see handles, bag opening, or thin layered sheets, do NOT classify as vinyl

box (cardboard):
- Flat, rigid panel structure with fold lines or creases
- Brown kraft or corrugated texture
- Box-like edges or flaps

paper (thin paper):
- Thin, flexible, matte surface (not shiny like plastic)
- Printed text or newspaper-like texture
- Paper fiber visible or tear patterns

glass:
- Transparent or semi-transparent rigid material
- Thick walls with visible glass edges
- Heavy-looking, smooth curved surface

unknown:
- Use this when evidence is unclear, mixed, or insufficient
- When the object could be two categories and you can't decide → unknown
- IMPORTANT: If you see ANY liquid, food residue, or leftover contents inside or on the object → ALWAYS classify as unknown, regardless of how clear the object type is

Now classify the object and respond with ONLY this JSON:
{{"category": "<category>", "confidence": "high/medium/low", "evidence": ["<clue 1>", "<clue 2>"], "counter_evidence": "<why it's not the next most likely category>"}} """

# [수정 포인트] confidence/evidence 기반 자동 unknown 처리 임계값
_MIN_EVIDENCE_COUNT = 2       # evidence가 이 수 미만이면 → unknown
_AUTO_UNKNOWN_CONFIDENCE = "low"  # 이 confidence면 → unknown

# 수정 (step2의 추측 값을 step3에 같이 넘겨서 참고해서 분류하도록)
def classify_object(client, bbox_image: np.ndarray, label_hint: str = "") -> int:
    """bbox 크롭 이미지 → type_id (0~6). 증거 부족 시 unknown(6)."""

    # ── [개선] Step 2 label 힌트 삽입 ──
    if label_hint:
        prompt = f'Step 2 identified this object as: "{label_hint}". Use this as a reference, but verify by examining the object yourself.\n\n' + _P3
    else:
        prompt = _P3
    
    resp = _call_gemini(client, bbox_image, prompt,
                        temperature=0.3,
                        thinking_budget=GEMINI_THINKING_BUDGET_CLASSIFY)
    # resp = _call_gemini(client, bbox_image, _P3,
    #                     temperature=0.3,
    #                     thinking_budget=GEMINI_THINKING_BUDGET_CLASSIFY)
    try:
        r = _parse_json(resp.text)
        raw = r["category"]
        conf = r.get("confidence", "?")
        evidence = r.get("evidence", [])
        counter = r.get("counter_evidence", "")

        # ── [안전장치] 증거 기반 자동 unknown 처리 ──────────
        auto_unknown = False
        reason = ""

        # 조건 1: confidence가 low면 → unknown
        if conf == _AUTO_UNKNOWN_CONFIDENCE:
            auto_unknown = True
            reason = f"confidence={conf}"

        # 조건 2: evidence 개수가 부족하면 → unknown
        if isinstance(evidence, list) and len(evidence) < _MIN_EVIDENCE_COUNT:
            auto_unknown = True
            reason = f"evidence {len(evidence)}개 < {_MIN_EVIDENCE_COUNT}개"

        if auto_unknown:
            print(f"[안전장치] '{raw}' → unknown 강제 변환 ({reason})")
            print(f"  evidence: {evidence}")
            print(f"  counter: {counter}")
            return CATEGORIES["unknown"]
        # ── 자동 unknown 끝 ────────────────────────────────

        # [개선 #4] word-boundary 매칭
        category = _match_category(raw)
        if category != raw.strip().lower():
            print(f"[보정] '{raw}' → '{category}' (word-boundary)")

        tid = CATEGORIES[category]
        print(f"[Step 3] {category} (id={tid}, conf={conf})")
        print(f"  evidence: {evidence}")
        print(f"  counter: {counter}")
        return tid
    except (json.JSONDecodeError, KeyError) as e:
        print(f"[에러] Step3: {e}")
        return CATEGORIES["unknown"]


# ── 보너스: 다수결 분류 ───────────────────────────────

def classify_with_consensus(client, bbox_image: np.ndarray,
                            n: int = 3) -> int:
    """n번 호출 → 다수결. API비용 n배이므로 정확도 중요할 때만."""
    from collections import Counter
    results = [classify_object(client, bbox_image) for _ in range(n)]
    winner, cnt = Counter(results).most_common(1)[0]
    print(f"[Consensus] {results} → id={winner} ({cnt}/{n})")
    return winner
