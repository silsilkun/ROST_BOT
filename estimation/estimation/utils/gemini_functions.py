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
_P2 = """Select the ONE waste object EASIEST to pick with a parallel gripper (isolated, clear edges).
Return ONLY JSON: {"box_2d":[ymin,xmin,ymax,xmax],"center":[cy,cx],"angle":<0-180>,"label":"<desc>"}
Coords normalized 0-1000. angle: 0=horizontal, 90=vertical, align shortest axis."""

def select_target_object(client, roi_image: np.ndarray) -> dict | None:
    """가장 집기 쉬운 객체 1개 → bbox, center(uv), angle. 실패→None."""
    resp = _call_gemini(client, roi_image, _P2,
                        temperature=GEMINI_TEMPERATURE,
                        thinking_budget=GEMINI_THINKING_BUDGET_SPATIAL)
    try:
        r = _parse_json(resp.text)
        for k in ("box_2d", "center", "angle"):                            # [안전장치] 필수키
            assert k in r, f"'{k}' 없음"
        for v in r["box_2d"] + r["center"]:                                # [안전장치] 좌표범위
            assert 0 <= v <= 1000, f"범위초과: {v}"
        out = {"bbox": r["box_2d"], "center": r["center"],
               "angle": float(r["angle"]), "label": r.get("label", "?")}
        print(f"[Step 2] '{out['label']}' center={out['center']} "
              f"angle={out['angle']}°")
        return out
    except (json.JSONDecodeError, KeyError, AssertionError) as e:
        print(f"[에러] Step2: {e}\n  원본: {resp.text}")
        return None


# ── Step 3: 카테고리 분류 ─────────────────────────────

# [수정 포인트] 분류 프롬프트 바꾸려면 여기만
_P3 = f"""Classify this waste into ONE category: {', '.join(CATEGORY_LIST)}
Guide: box=cardboard | paper=cups/newspapers | plastic=bottles/cups | vinyl=bags/wrappers | glass=bottles/jars | can=aluminum/tin | unknown=other
Return ONLY JSON: {{"category":"<cat>","confidence":"high/medium/low"}}"""

def classify_object(client, bbox_image: np.ndarray) -> int:
    """bbox 크롭 이미지 → type_id (0~6). 실패→unknown(6)."""
    resp = _call_gemini(client, bbox_image, _P3,
                        temperature=0.3,
                        thinking_budget=GEMINI_THINKING_BUDGET_CLASSIFY)
    try:
        r = _parse_json(resp.text)
        raw = r["category"]
        conf = r.get("confidence", "?")

        # [개선 #4] word-boundary 매칭
        category = _match_category(raw)
        if category != raw.strip().lower():
            print(f"[보정] '{raw}' → '{category}' (word-boundary)")

        tid = CATEGORIES[category]
        print(f"[Step 3] {category} (id={tid}, conf={conf})")
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
