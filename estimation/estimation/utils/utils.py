# utils.py (compact hardened)
import os, json, re, math
from typing import Optional, List, Dict, Tuple, Union

# ---------- parsing ----------
def parse_labels_from_text(text: str) -> List[str]:
    """Gemini 응답에서 labels(list[str])만 최대한 보수적으로 추출."""
    if not text: return []
    m = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL | re.IGNORECASE)
    s = (m.group(1) if m else text).strip()

    def _try_load(x: str):
        try: return json.loads(x)
        except json.JSONDecodeError: return None

    parsed = _try_load(s)
    if parsed is None:
        a, b = s.find("["), s.rfind("]")
        parsed = _try_load(s[a:b+1]) if (a != -1 and b > a) else None

    if isinstance(parsed, dict): parsed = parsed.get("labels")
    if isinstance(parsed, list):
        out = []
        for it in parsed:
            if isinstance(it, dict):
                v = it.get("label") or it.get("name") or it.get("type")
                if v is not None: out.append(str(v))
            else:
                out.append(str(it))
        # [보강] 흔한 케이스: "Plastic"/" CAN " 같은 변종 정리
        return [t.strip().lower() for t in out if str(t).strip()]

    # fallback: comma/newline split
    return [t.strip().lower() for t in re.split(r"[,\n]+", s) if t.strip()]

def convert_labels_to_ids(labels: List[str], label_map: Dict[str, float], unknown_id: float) -> List[float]:
    """
    1) exact match 우선
    2) 'plastic bottle'처럼 공백/특수문자 토큰 포함 케이스는 허용
    3) 'candy'가 'can'으로 매칭되는 오분류는 차단 (단어 경계 사용)
    """
    ids: List[float] = []
    keys = list(label_map.keys())

    for lb in (labels or []):
        norm = (lb or "").strip().lower()
        if not norm or norm == "unknown":
            ids.append(float(unknown_id))
            continue

        # exact match
        if norm in label_map:
            ids.append(float(label_map[norm]))
            continue

        # word-boundary match
        found = None
        for k in keys:
            pat = rf"\b{re.escape(k)}\b"
            if re.search(pat, norm):
                found = k
                break

        ids.append(float(label_map[found]) if found else float(unknown_id))

    return ids

# ---------- validation ----------
ValidateRet = Union[bool, Tuple[bool, str]]

def validate_waste_coordinates_flat(data: List[float]) -> ValidateRet:
    """[tmp_id,x,y,z,angle]*N (len=5N), NaN/Inf 방지. (bool 또는 (bool,reason))"""
    if not data or len(data) < 5 or (len(data) % 5 != 0):
        return False, "len must be 5N"
    try:
        for v in data:
            fv = float(v)
            if math.isnan(fv) or math.isinf(fv): return False, "NaN/Inf"
    except Exception as e:
        return False, f"non-numeric: {e}"
    return True, "ok"

# ---------- image io / encoding ----------
def read_image_file(path: str) -> Optional[bytes]:
    if not os.path.exists(path): return None
    try:
        with open(path, "rb") as f: return f.read()
    except OSError:
        return None

def guess_mime_type(name_or_fmt: str) -> str:
    s = (name_or_fmt or "").lower()
    if s.endswith(".png") or "png" in s: return "image/png"
    if s.endswith(".jpg") or s.endswith(".jpeg") or "jpeg" in s or "jpg" in s: return "image/jpeg"
    return "application/octet-stream"

def ros_image_to_bgr_numpy(bridge, ros_img_msg):
    """sensor_msgs/Image(encoding=bgr8) -> numpy BGR uint8"""
    # [수정 포인트] perception encoding 바뀌면 desired_encoding만 수정
    return bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding="bgr8")

def bgr_numpy_to_jpeg_bytes(bgr_img, jpeg_quality: int = 90) -> Optional[bytes]:
    """numpy BGR -> RGB -> JPEG bytes. 실패 시 None."""
    import cv2
    try:
        if bgr_img is None or not hasattr(bgr_img, "shape"): return None
        if len(bgr_img.shape) != 3 or bgr_img.shape[2] != 3: return None  # [보강] shape 방어
        if str(getattr(bgr_img, "dtype", "")) != "uint8":
            bgr_img = bgr_img.astype("uint8", copy=False)                  # [보강] dtype 방어
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        q = max(1, min(100, int(jpeg_quality)))                            # [보강] quality clamp
        ok, enc = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        return enc.tobytes() if ok else None
    except Exception:
        return None
