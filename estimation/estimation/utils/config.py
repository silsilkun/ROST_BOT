"""
R.O.S.T - 설정 파일 (config.py)
모든 상수/설정을 한 곳에서 관리한다.
다른 파일에서 from config import ... 로 가져다 쓴다.
"""

import os
from pathlib import Path

try:
    from dotenv import load_dotenv
except ImportError:
    load_dotenv = None


def _load_env_file_fallback(path: Path) -> None:
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#") or "=" not in s:
            continue
        key, value = s.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


# utils/.env를 자동 로드해서 실행 환경마다 별도 export 없이 사용 가능하게 한다.
_this_dir = Path(__file__).resolve().parent
_env_candidates = [_this_dir / ".env"]

for parent in _this_dir.parents:
    _env_candidates.append(parent / "src" / "estimation" / "estimation" / "utils" / ".env")

for _env_path in _env_candidates:
    if _env_path.exists():
        if load_dotenv is not None:
            load_dotenv(_env_path)
        else:
            _load_env_file_fallback(_env_path)
        break

# ── Gemini API ──────────────────────────────────────────
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE")
GEMINI_MODEL = "gemini-robotics-er-1.5-preview"

GEMINI_TEMPERATURE = 0.5
# Step 1,2: 공간 추론 → thinking 낮게 (빠름)
GEMINI_THINKING_BUDGET_SPATIAL = 0
# Step 3: 분류 추론 → thinking 높게 (정확)
GEMINI_THINKING_BUDGET_CLASSIFY = 1024
# Gemini 좌표 정규화 범위 (문서 기준 0~1000)
GEMINI_COORD_RANGE = 1000

# ── 집기 각도/폭 보정 ──────────────────────────────────
PREFER_GRASP_PTS_ANGLE = False
USE_DIRECT_TARGET_PICK = False
GRIPPER_ANGLE_USE_GRASP_NORMAL = False
GRIPPER_ANGLE_OFFSET_DEG_CW = 0.0
USE_COMPLEMENTARY_GRIPPER_ANGLE = False
USE_OBJECT_CONTOUR_GEOMETRY = False
PREFER_GRASP_PTS_SHORT_SIDE = False
SHORT_SIDE_MM_PER_PX_GRASP = 1.0
SHORT_SIDE_MM_BIAS_GRASP = 20.0
SHORT_SIDE_MM_PER_PX_AABB = 0.6
SHORT_SIDE_MM_BIAS_AABB = 0.0

# ── 카테고리 매핑 ───────────────────────────────────────
CATEGORIES = {
    "box": 0, 
    "paper": 1,
    "plastic": 2,
    "vinyl": 3,
    "glass": 4,
    "can": 5,   
    "unknown": 6,
}

# 카테고리 이름 리스트
CATEGORY_LIST = list(CATEGORIES.keys())

# ── Bin 위치 로딩 (환경변수 → 기본값) ───────────────────
def _env_float(key: str, default: float = 0.0) -> float:
    try:
        return float(os.environ.get(key, default))
    except (TypeError, ValueError):
        return float(default)

tx0, ty0 = _env_float("BIN_BOX_X"), _env_float("BIN_BOX_Y")
tx1, ty1 = _env_float("BIN_PAPER_X"), _env_float("BIN_PAPER_Y")
tx2, ty2 = _env_float("BIN_PLASTIC_X"), _env_float("BIN_PLASTIC_Y")
tx3, ty3 = _env_float("BIN_VINYL_X"), _env_float("BIN_VINYL_Y")
tx4, ty4 = _env_float("BIN_GLASS_X"), _env_float("BIN_GLASS_Y")
tx5, ty5 = _env_float("BIN_CAN_X"), _env_float("BINㅊ_CAN_Y")
tx6, ty6 = _env_float("BIN_UNKNOWN_X"), _env_float("BIN_UNKNOWN_Y")

# ── Bin 위치 (로봇 좌표, mm) ──────────────────────────
BIN_POSITIONS = {
    "box":     (750, 350),
    "paper":   (550, 350),
    "plastic": (350, 350),
    "vinyl":   (350, -300),
    "glass":   (520, -300),
    "can":     (720, -330),
    "unknown": (670, 20),
}

# ── RealSense 카메라 ───────────────────────────────────
REALSENSE_WIDTH = 1280
REALSENSE_HEIGHT = 720
REALSENSE_FPS = 30

# ── 안전장치: 설정값 검증 ──────────────────────────────
assert GEMINI_API_KEY != "", "API 키가 비어있습니다"
assert REALSENSE_WIDTH > 0 and REALSENSE_HEIGHT > 0, "카메라 해상도 이상"
assert "unknown" in CATEGORIES, "unknown 카테고리는 반드시 있어야 합니다"
assert all(isinstance(v, int) for v in CATEGORIES.values()), "type_id는 정수여야 합니다"
