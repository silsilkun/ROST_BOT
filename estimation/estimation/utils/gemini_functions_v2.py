"""
R.O.S.T - Gemini API 함수 (gemini_functions.py)  ★ 핵심 모듈

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
)


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
_MAX_RETRIES = 1
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
{"box_2d": [ymin, xmin, ymax, xmax], "center": [cy, cx], "angle": <0-180>, "label": "<short description>"}

Rules for the JSON values:
- All coordinates are normalized 0 to 1000.
- "center" must be exactly 2 numbers: the y and x of the object's center point. Not 4, just 2.
- "angle" is the rotation for a gripper: 0 means horizontal, 90 means vertical. Align with the object's shortest axis.
- "label" is a brief description like "crushed plastic cup" or "tomato can".
- Every field is required. Always include box_2d, center, angle, and label."""


def select_target_object(client, roi_image: np.ndarray) -> dict | None:
    """가장 집기 쉬운 객체 1개 → bbox, center(uv), angle. 실패→None."""
    resp = _call_gemini(client, roi_image, _P2,
                        temperature=GEMINI_TEMPERATURE,
                        thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
    try:
        r = _parse_json(resp.text)

        # ── [안전장치] center 누락 시 box_2d에서 자동 계산 ──
        if "center" not in r and "box_2d" in r:
            ymin, xmin, ymax, xmax = r["box_2d"]
            r["center"] = [(ymin + ymax) // 2, (xmin + xmax) // 2]
            print(f"[보정] center 누락 → box_2d에서 계산: {r['center']}")

        # ── [안전장치] angle 누락 시 기본값 0 ──
        if "angle" not in r:
            r["angle"] = 0
            print(f"[보정] angle 누락 → 기본값 0°")

        for k in ("box_2d", "center", "angle"):                            # [안전장치] 필수키
            assert k in r, f"'{k}' 없음"

        # ── [안전장치] center 값 방어 ──────────────────────
        center = r["center"]
        if isinstance(center, list) and len(center) == 4:
            # 4개 → 중심점 계산
            cy = (center[0] + center[2]) // 2
            cx = (center[1] + center[3]) // 2
            center = [cy, cx]
            print(f"[보정] center 4개→2개 변환: {r['center']} → {center}")
        elif isinstance(center, list) and len(center) == 2:
            # 값이 숫자인지 확인 (문자열 "cy" 같은 거 방어)
            try:
                center = [int(center[0]), int(center[1])]
            except (ValueError, TypeError):
                print(f"[에러] center 값이 숫자가 아님: {center}")
                return None
        else:
            print(f"[에러] center 형식 이상: {center}")
            return None
        # ── center 방어 끝 ─────────────────────────────────

        for v in r["box_2d"] + center:                                     # [안전장치] 좌표범위
            assert 0 <= v <= 1000, f"범위초과: {v}"

        out = {"bbox": r["box_2d"], "center": center,
               "angle": float(r["angle"]), "label": r.get("label", "?")}
        print(f"[Step 2] '{out['label']}' center={out['center']} "
              f"angle={out['angle']}°")
        return out
    except (json.JSONDecodeError, KeyError, AssertionError) as e:
        print(f"[에러] Step2: {e}\n  원본: {resp.text}")
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
