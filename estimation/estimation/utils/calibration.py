"""
R.O.S.T - 캘리브레이션 (calibration.py)
uv 좌표(이미지 픽셀) → 로봇 좌표(로봇팔이 실제 이동할 좌표) 변환

⚠️ PLACEHOLDER — 파트장님 복귀 후 실제 변환 행렬 채워넣기

[액션 아이템]
1. 기존 캘리브레이션 코드 + 변환 행렬 파일 공유받기
2. 카메라 위치/각도가 데모 때와 동일한지 확인 → 다르면 재캘리브레이션
3. 재캘리브레이션 시: ArUco 배치 → 포인트 쌍 수집 → 행렬 재계산
4. 아래 placeholder에 실제 로직 채워넣기
5. 최소 5개 포인트에서 오차 측정 검증
"""

import numpy as np
from config import GEMINI_COORD_RANGE


def load_transform_matrix(filepath: str = None) -> np.ndarray:
    """
    변환 행렬 로드. 없으면 단위 행렬(변환 없음) 반환.
    [수정 포인트] 파트장님 캘리브레이션 완료 후 filepath 지정
    """
    if filepath is not None:
        try:
            matrix = np.load(filepath)
            print(f"[캘리브레이션] 행렬 로드: {filepath} (shape={matrix.shape})")
            # [안전장치] 행렬 크기 검증
            assert matrix.shape in ((3, 3), (4, 4)), \
                f"변환 행렬 shape이 이상합니다: {matrix.shape}"
            return matrix
        except FileNotFoundError:
            print(f"[에러] 파일 없음: {filepath}")

    print("[경고] 변환 행렬 없음 → 단위 행렬 사용 (캘리브레이션 필요)")
    return np.eye(3)


def gemini_to_pixel(center_normalized: list, roi: tuple) -> tuple:
    """
    Gemini 정규화 좌표(0~1000) → 전체 이미지 픽셀 좌표.
    center_normalized: [cy, cx] (Gemini 출력, y먼저 x나중)
    roi: (roi_x, roi_y, roi_w, roi_h)
    """
    cy_norm, cx_norm = center_normalized
    roi_x, roi_y, roi_w, roi_h = roi

    # [수정 포인트] 정규화 범위가 바뀌면 config.py의 GEMINI_COORD_RANGE만 수정
    local_x = int(cx_norm / GEMINI_COORD_RANGE * roi_w)
    local_y = int(cy_norm / GEMINI_COORD_RANGE * roi_h)

    # ROI offset 더하기 → 전체 이미지 기준
    pixel_u = roi_x + local_x
    pixel_v = roi_y + local_y
    return (pixel_u, pixel_v)


def pixel_to_robot(pixel_u: int, pixel_v: int,
                   transform_matrix: np.ndarray) -> tuple:
    """
    이미지 픽셀 좌표(u,v) → 로봇 좌표(tx,ty).
    [PLACEHOLDER] 변환 행렬이 실제 값이어야 정확한 좌표가 나온다.
    """
    uv_h = np.array([pixel_u, pixel_v, 1.0])      # 동차 좌표
    robot_h = transform_matrix @ uv_h              # 행렬 곱
    tx = float(robot_h[0] / robot_h[2])            # 동차 → 데카르트
    ty = float(robot_h[1] / robot_h[2])
    return (tx, ty)


def uv_to_robot_coords(center_normalized: list, roi: tuple,
                        transform_matrix: np.ndarray) -> tuple:
    """
    Gemini 좌표 → 로봇 좌표. 한 번에 변환.
    gemini_to_pixel + pixel_to_robot 순차 호출.
    """
    pixel_u, pixel_v = gemini_to_pixel(center_normalized, roi)
    tx, ty = pixel_to_robot(pixel_u, pixel_v, transform_matrix)
    print(f"[좌표] Gemini{center_normalized} → px({pixel_u},{pixel_v}) → robot({tx:.2f},{ty:.2f})")
    return (tx, ty)
