"""
R.O.S.T - 설정 파일 (config.py)
모든 상수/설정을 한 곳에서 관리한다.
다른 파일에서 from config import ... 로 가져다 쓴다.
"""

import os

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

# ── 집기 각도/폭 보정 ──────────────────────────────────
# Step2 각도는 grasp_pts 기반이 현장 기준과 더 잘 맞는다.
PREFER_GRASP_PTS_ANGLE = False
# Step2 direct(center+grasp_pts) 우선 사용 여부
USE_DIRECT_TARGET_PICK = False
# grasp_pts 선분에 수직한 방향을 그리퍼 yaw로 사용할지 여부
GRIPPER_ANGLE_USE_GRASP_NORMAL = False #True
# 로봇 각도 미세 보정값(시계방향 +deg, 반시계방향 -deg)
# (complementary 보정 사용 시 0부터 시작해 미세조정)
GRIPPER_ANGLE_OFFSET_DEG_CW = 0.0
# 로봇 각도 기준이 반대 축이면 180-angle 보정 사용
USE_COMPLEMENTARY_GRIPPER_ANGLE = False #True
# 물체 윤곽(contour) 기반 길이/각도 산출 사용
USE_OBJECT_CONTOUR_GEOMETRY = False #True
# short_side는 corners 기반 우선 (grasp_pts는 신뢰도 통과 시만 사용)
PREFER_GRASP_PTS_SHORT_SIDE = False
# short_side 길이 보정 (현장 실측 기준)
# OBB(grasp_pts) 기준: 42px -> 62mm
SHORT_SIDE_MM_PER_PX_GRASP = 1.0
SHORT_SIDE_MM_BIAS_GRASP = 20.0
# AABB fallback 기준: 104px -> 62mm
SHORT_SIDE_MM_PER_PX_AABB = 0.6
SHORT_SIDE_MM_BIAS_AABB = 0.0

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

# ── Bin 위치 (로봇 좌표, mm) ──────────────────────────
BIN_POSITIONS = {
    "box":     (tx0, ty0),
    "paper":   (tx1, ty1),
    "plastic": (tx2, ty2),
    "vinyl":   (tx3, ty3),
    "glass":   (tx4, ty4),
    "can":     (tx5, ty5),
    "unknown": (tx6, ty6),
}

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
