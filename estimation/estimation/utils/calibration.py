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

# ── 픽셀→로봇 보정 (수동 캘리브레이션 3점) ──────────────
# [u, v, 1] @ A = [tx, ty]  (단위: mm)
USE_PIXEL_AFFINE = True
PIXEL_TO_ROBOT_AFFINE = np.array([
    [-2.55879128e-03,  1.15706105e+00],
    [ 1.03691970e+00, -1.23309370e-01],
    [ 2.46053004e+02, -6.93681979e+02],
], dtype=float)


def load_transform_matrix(filepath: str = None):
    """
    캘리브레이션 데이터 로드.
    - camcalib.npz: T_cam_to_work(4x4), camera_matrix(3x3) 사용
    - npy/npz 단일 3x3/4x4 행렬: 그대로 사용
    없으면 단위 행렬(변환 없음) 반환.
    """
    if filepath is not None:
        try:
            data = np.load(filepath)
            # npz일 경우
            if isinstance(data, np.lib.npyio.NpzFile):
                if "T_cam_to_work" in data and "camera_matrix" in data:
                    T = data["T_cam_to_work"]
                    K = data["camera_matrix"]
                    print(f"[캘리브레이션] camcalib 로드: {filepath} (T:{T.shape}, K:{K.shape})")
                    return {"T_cam_to_work": T, "camera_matrix": K}
                # 단일 행렬 저장 케이스
                if len(data.files) == 1:
                    matrix = data[data.files[0]]
                    print(f"[캘리브레이션] 행렬 로드: {filepath} (shape={matrix.shape})")
                    assert matrix.shape in ((3, 3), (4, 4)), \
                        f"변환 행렬 shape이 이상합니다: {matrix.shape}"
                    return matrix
                print(f"[경고] 알 수 없는 npz 구조: {data.files}")
            else:
                matrix = data
                print(f"[캘리브레이션] 행렬 로드: {filepath} (shape={matrix.shape})")
                assert matrix.shape in ((3, 3), (4, 4)), \
                    f"변환 행렬 shape이 이상합니다: {matrix.shape}"
                return matrix
        except FileNotFoundError:
            print(f"[에러] 파일 없음: {filepath}")

    print("[경고] 캘리브레이션 없음 → 단위 행렬 사용 (정확도 저하)")
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


def pixel_to_robot(pixel_u: int, pixel_v: int, transform_matrix) -> tuple:
    """
    이미지 픽셀 좌표(u,v) → 로봇 좌표(tx,ty).
    - camcalib(dict): K + T_cam_to_work 사용, z=0 평면과의 교차로 계산
    - 3x3/4x4 행렬: 기존 동차 좌표 변환
    """
    if USE_PIXEL_AFFINE:
        uv1 = np.array([pixel_u, pixel_v, 1.0], dtype=float)
        tx, ty = uv1 @ PIXEL_TO_ROBOT_AFFINE
        return (float(tx), float(ty))
    # camcalib.npz 기반 (K, T)
    if isinstance(transform_matrix, dict):
        K = transform_matrix["camera_matrix"]
        T = transform_matrix["T_cam_to_work"]
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # 픽셀 → 카메라 좌표계 광선
        x = (pixel_u - cx) / fx
        y = (pixel_v - cy) / fy
        ray_cam = np.array([x, y, 1.0], dtype=float)

        # 카메라 → 작업좌표계 변환
        R = T[:3, :3]
        t = T[:3, 3]
        origin_w = t
        dir_w = R @ ray_cam

        # 작업좌표계 z=0 평면과의 교차
        if abs(dir_w[2]) < 1e-6:
            print("[경고] 광선이 평면과 평행 → 좌표 계산 실패")
            return (0.0, 0.0)
        s = -origin_w[2] / dir_w[2]
        point_w = origin_w + s * dir_w
        # camcalib 기준은 cm로 저장되어 있음 → 최종 출력은 mm
        tx, ty = float(point_w[0] * 10.0), float(point_w[1] * 10.0)
        return (tx, ty)

    # 기존 3x3/4x4 행렬 케이스
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
