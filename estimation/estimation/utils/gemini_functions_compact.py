"""
R.O.S.T - Gemini API (gemini_functions_compact.py)  ★ 150줄 이내 압축본
동일 기능 + 4개 개선사항(다단계파싱, 이미지방어, retry백오프, word-boundary) 포함
"""
import json, re, time, cv2, numpy as np
from google import genai
from google.genai import types
from config import (GEMINI_API_KEY, GEMINI_MODEL, GEMINI_TEMPERATURE,
                    GEMINI_THINKING_BUDGET_SPATIAL, GEMINI_THINKING_BUDGET_CLASSIFY,
                    CATEGORY_LIST, CATEGORIES)

# ── 공통 유틸 ─────────────────────────────────────────
def init_gemini_client():
    assert GEMINI_API_KEY != "YOUR_API_KEY_HERE", "config.py에서 API키 설정"
    c = genai.Client(api_key=GEMINI_API_KEY); print(f"[Gemini] 초기화 ({GEMINI_MODEL})"); return c

def _to_bytes(img: np.ndarray) -> bytes:
    """[개선#2] shape/dtype 방어"""
    if img is None or not hasattr(img, "shape"): raise ValueError("이미지 None")
    if len(img.shape) != 3 or img.shape[2] != 3: raise ValueError(f"BGR 필요: {img.shape}")
    if img.dtype != np.uint8: img = img.astype(np.uint8, copy=False)
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])  # [수정 포인트] 품질
    if not ok: raise RuntimeError("JPEG 인코딩 실패")
    return buf.tobytes()

def _parse_json(text: str):
    """[개선#1] 다단계 fallback: 코드펜스→그대로→{}추출→[]추출"""
    if not (text or "").strip(): raise json.JSONDecodeError("빈 응답", text or "", 0)
    c = text.strip()
    m = re.search(r"```(?:json)?\s*(.*?)```", c, re.DOTALL)
    if m: c = m.group(1).strip()
    for extract in [lambda s: s,
                    lambda s: s[s.find("{"):s.rfind("}")+1] if "{" in s else None,
                    lambda s: s[s.find("["):s.rfind("]")+1] if "[" in s else None]:
        try:
            chunk = extract(c)
            if chunk: return json.loads(chunk)
        except (json.JSONDecodeError, TypeError): pass
    raise json.JSONDecodeError(f"파싱불가(len={len(text)})", text, 0)

_RETRIES = 1; _BACKOFF = 0.5  # [수정 포인트] 재시도 횟수/대기

def _call(client, img, prompt, temp, think):
    """[개선#3] retry + 지수 백오프 + 빈 응답 재시도"""
    ib = _to_bytes(img); err = None
    for i in range(_RETRIES + 1):
        try:
            r = client.models.generate_content(model=GEMINI_MODEL,
                contents=[types.Part.from_bytes(data=ib, mime_type='image/jpeg'), prompt],
                config=types.GenerateContentConfig(temperature=temp,
                    thinking_config=types.ThinkingConfig(thinking_budget=think)))
            if not (r.text or "").strip(): raise RuntimeError("빈 응답")
            return r
        except Exception as e:
            err = e
            if i < _RETRIES:
                w = _BACKOFF*(2**i); print(f"[Gemini] 재시도 {i+1} ({w:.1f}s): {e}"); time.sleep(w)
    raise RuntimeError(f"Gemini {_RETRIES+1}회 실패: {err}")

def _match_cat(label: str) -> str:
    """[개선#4] word-boundary 매칭. 'candy'→'can' 차단."""
    n = (label or "").strip().lower()
    if not n or n == "unknown": return "unknown"
    if n in CATEGORIES: return n
    for k in CATEGORIES:
        if re.search(rf"\b{re.escape(k)}\b", n): return k
    return "unknown"

# ── Step 1: 객체 존재 확인 ────────────────────────────
def check_objects_exist(client, roi_image: np.ndarray) -> bool:
    """ROI 안 쓰레기 유무. False → 루프 종료."""
    r = _call(client, roi_image,
        "Look at this waste collection area. Is there any waste remaining? Answer ONLY 'yes' or 'no'.",
        temp=0.1, think=GEMINI_THINKING_BUDGET_SPATIAL)
    ok = "yes" in r.text.strip().lower()
    print(f"[Step 1] {'있음 ✓' if ok else '없음 → 종료'}"); return ok

# ── Step 2: 타겟 선정 ────────────────────────────────
# [수정 포인트] 프롬프트 바꾸려면 여기만
_P2 = """Select ONE waste object EASIEST to pick (isolated, clear edges).
Return ONLY JSON: {"box_2d":[ymin,xmin,ymax,xmax],"center":[cy,cx],"angle":<0-180>,"label":"<desc>"}
Coords 0-1000. angle: 0=horizontal, 90=vertical."""

def select_target_object(client, roi_image: np.ndarray) -> dict | None:
    """가장 집기 쉬운 객체 1개 → bbox, center(uv), angle. 실패→None."""
    r = _call(client, roi_image, _P2, temp=GEMINI_TEMPERATURE, think=GEMINI_THINKING_BUDGET_SPATIAL)
    try:
        d = _parse_json(r.text)
        for k in ("box_2d","center","angle"): assert k in d, f"'{k}' 없음"
        for v in d["box_2d"]+d["center"]: assert 0<=v<=1000, f"범위초과:{v}"
        o = {"bbox":d["box_2d"],"center":d["center"],"angle":float(d["angle"]),"label":d.get("label","?")}
        print(f"[Step 2] '{o['label']}' c={o['center']} a={o['angle']}°"); return o
    except (json.JSONDecodeError, KeyError, AssertionError) as e:
        print(f"[에러] Step2: {e}"); return None

# ── Step 3: 카테고리 분류 ─────────────────────────────
# [수정 포인트] 분류 프롬프트 바꾸려면 여기만
_P3 = f"""Classify this waste into ONE: {', '.join(CATEGORY_LIST)}
box=cardboard|paper=cups/newspapers|plastic=bottles/cups|vinyl=bags/wrappers|glass=bottles/jars|can=aluminum/tin|unknown=other
Return ONLY JSON: {{"category":"<cat>","confidence":"high/medium/low"}}"""

def classify_object(client, bbox_image: np.ndarray) -> int:
    """bbox 크롭 → type_id(0~6). 실패→unknown(6)."""
    r = _call(client, bbox_image, _P3, temp=0.3, think=GEMINI_THINKING_BUDGET_CLASSIFY)
    try:
        d = _parse_json(r.text); raw = d["category"]; conf = d.get("confidence","?")
        cat = _match_cat(raw)
        if cat != raw.strip().lower(): print(f"[보정] '{raw}'→'{cat}'")
        tid = CATEGORIES[cat]; print(f"[Step 3] {cat} (id={tid}, {conf})"); return tid
    except (json.JSONDecodeError, KeyError) as e:
        print(f"[에러] Step3: {e}"); return CATEGORIES["unknown"]

# ── 다수결 분류 ───────────────────────────────────────
def classify_with_consensus(client, bbox_image: np.ndarray, n: int = 3) -> int:
    """n번 호출 다수결. 정확도 중요할 때만."""
    from collections import Counter
    res = [classify_object(client, bbox_image) for _ in range(n)]
    w, c = Counter(res).most_common(1)[0]; print(f"[Consensus] {res}→id={w}({c}/{n})"); return w
