"""
R.O.S.T - 좌표 변환 (calibration.py)
카메라 픽셀 좌표 → 로봇 작업 좌표 변환

기반: perception 파트장님의 coordinate.py (1차 데모 검증 완료)
변환 순서:
  Gemini 정규화(0~1000) → 픽셀(u,v) → 왜곡 보정 → 카메라 3D → 로봇 좌표

필요 파일: camcalib.npz (같은 폴더에 위치)
  - T_cam_to_work: 카메라→작업좌표 변환 행렬 (4x4)
  - camera_matrix: 카메라 내부 행렬 K (3x3)
  - dist_coeffs: 렌즈 왜곡 계수 D
"""

import os
import numpy as np
import cv2
from config import (
    CALIB_NPZ, DEPTH_MIN_M, DEPTH_MAX_M,
    DEPTH_SAMPLE_RADIUS, OFFSET_X, OFFSET_Y, OFFSET_Z,
    GEMINI_COORD_RANGE,
)


# ── 캘리브레이션 데이터 캐시 ──────────────────────────
_CALIB_CACHE = None


def _load_calib():
    """
    npz 파일에서 캘리브레이션 데이터 로드.
    한 번 로드하면 캐시해서 재사용한다.
    """
    global _CALIB_CACHE
    if _CALIB_CACHE is not None:
        return _CALIB_CACHE

    # [수정 포인트] npz 경로가 다르면 config.py의 CALIB_NPZ만 수정
    base = os.path.dirname(os.path.abspath(__file__))
    path = CALIB_NPZ
    if not os.path.isabs(path):
        path = os.path.join(base, path)

    # [안전장치] 파일 존재 확인
    if not os.path.exists(path):
        print(f"[경고] 캘리브 파일 없음: {path}")
        print("       → 더미 변환 사용 (좌표 정확도 보장 안 됨)")
        return None

    data = np.load(path)
    _CALIB_CACHE = {
        "T": data["T_cam_to_work"].astype(np.float64),
        "K": data["camera_matrix"].astype(np.float64),
        "D": data["dist_coeffs"].astype(np.float64),
    }
    print(f"[캘리브] 로드 완료: {path}")
    return _CALIB_CACHE


def pixel_to_robot(u: int, v: int, depth_map_m: np.ndarray):
    """
    픽셀 좌표(u, v) + depth map → 로봇 작업 좌표 (x, y, z).

    비유: "사진 속 위치"를 "로봇 팔이 가야 할 실제 위치"로 번역.
      1단계: 주변 5×5 영역에서 depth 중앙값 (노이즈 제거)
      2단계: 렌즈 왜곡 보정 (fisheye 효과 제거)
      3단계: 2D→3D 복원 (depth로 깊이 추가)
      4단계: 카메라 좌표 → 로봇 좌표 (T 행렬 적용)
      5단계: 오프셋 보정 (미세 조정)

    Returns: (tx, ty, tz) 로봇 좌표 (cm), 실패 시 None
    """
    calib = _load_calib()
    if calib is None:
        # 캘리브 파일 없으면 더미 반환
        print("[경고] 캘리브 없음 → 픽셀 좌표 그대로 반환")
        return float(u), float(v), 0.0

    T, K, D = calib["T"], calib["K"], calib["D"]
    H, W = depth_map_m.shape[:2]

    # [안전장치] 좌표 클램프
    u = int(np.clip(u, 0, W - 1))
    v = int(np.clip(v, 0, H - 1))

    # ── 1단계: depth 수집 (주변 영역 중앙값) ──────────
    r = DEPTH_SAMPLE_RADIUS
    depths = []
    for du in range(-r, r + 1):
        for dv in range(-r, r + 1):
            uu, vv = u + du, v + dv
            if 0 <= uu < W and 0 <= vv < H:
                d = float(depth_map_m[vv, uu])
                if d > 0.0 and DEPTH_MIN_M <= d <= DEPTH_MAX_M:
                    depths.append(d)

    if not depths:
        print(f"[경고] 유효한 depth 없음 (u={u}, v={v})")
        return None

    Z_cm = float(np.median(depths)) * 100.0  # 미터 → cm

    # ── 2단계: 렌즈 왜곡 보정 ─────────────────────────
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    pts = np.array([[[u, v]]], dtype=np.float32)
    und = cv2.undistortPoints(pts, K, D, P=K)
    uc, vc = float(und[0, 0, 0]), float(und[0, 0, 1])

    # ── 3단계: 2D→3D 카메라 좌표 ─────────────────────
    # v9 규약: u→Yc, v→Xc (카메라 축 방향 주의)
    Yc = (uc - cx) * Z_cm / fx
    Xc = (vc - cy) * Z_cm / fy
    Pc = np.array([Xc, Yc, Z_cm, 1.0], dtype=np.float64)

    # ── 4단계: 카메라 → 로봇 좌표 변환 ───────────────
    Pw = T @ Pc

    # ── 5단계: 오프셋 보정 ────────────────────────────
    # [수정 포인트] 로봇 위치가 바뀌면 config.py의 OFFSET 값 수정
    Pw[0] = -1 * Pw[0] + OFFSET_X
    Pw[1] = -1 * Pw[1] + OFFSET_Y
    Pw[2] = -1 * Pw[2] + OFFSET_Z

    return float(Pw[0]), float(Pw[1]), float(Pw[2])


def gemini_to_pixel(center_normalized: list, roi: tuple) -> tuple:
    """
    Gemini 정규화 좌표(0~1000) → 전체 이미지 픽셀 좌표.

    center_normalized: [cy, cx] (Gemini 형식)
    roi: (rx, ry, rw, rh) — ROI 오프셋
    Returns: (u, v) 전체 이미지 기준 픽셀 좌표
    """
    cy, cx = center_normalized
    rx, ry, rw, rh = roi

    # 정규화 → ROI 내 픽셀
    px_in_roi = int(cx / GEMINI_COORD_RANGE * rw)
    py_in_roi = int(cy / GEMINI_COORD_RANGE * rh)

    # ROI 오프셋 → 전체 이미지 픽셀
    u = rx + px_in_roi
    v = ry + py_in_roi

    return u, v


def gemini_to_robot(center_normalized: list, roi: tuple,
                    depth_map_m: np.ndarray):
    """
    Gemini 좌표 → 로봇 좌표. 전체 파이프라인 통합 함수.

    center_normalized: [cy, cx] (Gemini)
    roi: (rx, ry, rw, rh)
    depth_map_m: (H, W) 미터 단위 depth map
    Returns: (tx, ty, tz) 로봇 좌표 (cm), 실패 시 None
    """
    u, v = gemini_to_pixel(center_normalized, roi)
    result = pixel_to_robot(u, v, depth_map_m)
    if result is None:
        print(f"[경고] 좌표 변환 실패 (u={u}, v={v})")
        return None
    tx, ty, tz = result
    print(f"[좌표] Gemini{center_normalized} → pixel({u},{v}) → "
          f"robot({tx:.1f}, {ty:.1f}, {tz:.1f}) cm")
    return tx, ty, tz
