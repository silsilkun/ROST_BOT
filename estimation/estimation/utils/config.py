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
    """python-dotenv가 없을 때 최소 파서로 KEY=VALUE를 로드한다."""
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
# 설치본(install)에서 실행할 때도 src 경로의 .env를 찾을 수 있게 후보 경로를 순회한다.
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
# [수정 포인트] API 키는 환경변수로 관리. 없으면 직접 입력.
GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE")
# [수정 포인트] 모델명이 바뀌면 여기만 수정
GEMINI_MODEL = "gemini-robotics-er-1.5-preview"

GEMINI_TEMPERATURE = 0.5
# Step 1,2: 공간 추론 → thinking 낮게 (빠름)
GEMINI_THINKING_BUDGET_SPATIAL = 0
# Step 3: 분류 추론 → thinking 높게 (정확)
GEMINI_THINKING_BUDGET_CLASSIFY = 1024

# Gemini 좌표 정규화 범위 (문서 기준 0~1000)
GEMINI_COORD_RANGE = 1000

# ── 카테고리 매핑 ───────────────────────────────────────
# [수정 포인트] 카테고리를 추가/삭제하면 여기만 수정
CATEGORIES = {
    "box": 0,      # 박스/종이박스
    "paper": 1,    # 종이
    "plastic": 2,  # 플라스틱
    "vinyl": 3,    # 비닐
    "glass": 4,    # 유리
    "can": 5,      # 캔
    "unknown": 6,  # 미분류
}

# 카테고리 이름 리스트 (Gemini 프롬프트에 전달용)
CATEGORY_LIST = list(CATEGORIES.keys())

# ── RealSense 카메라 ───────────────────────────────────
# [수정 포인트] 해상도/FPS 바꾸면 여기만 수정
REALSENSE_WIDTH = 1280
REALSENSE_HEIGHT = 720
REALSENSE_FPS = 30

# ── 안전장치: 설정값 검증 ──────────────────────────────
assert GEMINI_API_KEY != "", "API 키가 비어있습니다"
assert REALSENSE_WIDTH > 0 and REALSENSE_HEIGHT > 0, "카메라 해상도 이상"
assert "unknown" in CATEGORIES, "unknown 카테고리는 반드시 있어야 합니다"
assert all(isinstance(v, int) for v in CATEGORIES.values()), "type_id는 정수여야 합니다"
