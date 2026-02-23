"""
R.O.S.T - 좌표 변환 (calibration.py)
카메라 픽셀 좌표 -> 로봇 작업 좌표 변환
"""

import os
from pathlib import Path

import cv2
import numpy as np

from estimation.utils.config import (
    CALIB_NPZ,
    DEPTH_MIN_M,
    DEPTH_MAX_M,
    DEPTH_SAMPLE_RADIUS,
    OFFSET_X,
    OFFSET_Y,
    OFFSET_Z,
    GEMINI_COORD_RANGE,
)

_CALIB_CACHE = None


def _candidate_calib_paths() -> list[Path]:
    this_dir = Path(__file__).resolve().parent
    candidates = []

    path = Path(CALIB_NPZ)
    if path.is_absolute():
        candidates.append(path)
    else:
        candidates.append(this_dir / path)
        for parent in this_dir.parents:
            candidates.append(parent / "src" / "estimation" / "estimation" / "utils" / path)

    # 중복 제거(순서 유지)
    seen = set()
    uniq = []
    for p in candidates:
        s = str(p)
        if s not in seen:
            seen.add(s)
            uniq.append(p)
    return uniq


def _load_calib():
    """npz 파일에서 캘리브레이션 데이터를 로드하고 캐시한다."""
    global _CALIB_CACHE
    if _CALIB_CACHE is not None:
        return _CALIB_CACHE

    for path in _candidate_calib_paths():
        if path.exists():
            data = np.load(path)
            _CALIB_CACHE = {
                "T": data["T_cam_to_work"].astype(np.float64),
                "K": data["camera_matrix"].astype(np.float64),
                "D": data["dist_coeffs"].astype(np.float64),
            }
            print(f"[캘리브] 로드 완료: {path}")
            return _CALIB_CACHE

    print(f"[에러] 캘리브 파일 없음: {CALIB_NPZ}")
    return None


def pixel_to_robot(u: int, v: int, depth_map_m: np.ndarray):
    """
    픽셀 좌표(u,v) + depth map -> 로봇 좌표(tx, ty, tz) [cm]
    실패 시 None 반환.
    """
    calib = _load_calib()
    if calib is None:
        print("[에러] 캘리브 데이터 없음 -> 좌표 변환 중지")
        return None

    T, K, D = calib["T"], calib["K"], calib["D"]
    h, w = depth_map_m.shape[:2]

    u = int(np.clip(u, 0, w - 1))
    v = int(np.clip(v, 0, h - 1))

    r = DEPTH_SAMPLE_RADIUS
    depths = []
    for du in range(-r, r + 1):
        for dv in range(-r, r + 1):
            uu, vv = u + du, v + dv
            if 0 <= uu < w and 0 <= vv < h:
                d = float(depth_map_m[vv, uu])
                if d > 0.0 and DEPTH_MIN_M <= d <= DEPTH_MAX_M:
                    depths.append(d)

    if not depths:
        print(f"[경고] 유효한 depth 없음 (u={u}, v={v})")
        return None

    z_cm = float(np.median(depths)) * 100.0

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    pts = np.array([[[u, v]]], dtype=np.float32)
    und = cv2.undistortPoints(pts, K, D, P=K)
    uc, vc = float(und[0, 0, 0]), float(und[0, 0, 1])

    yc = (uc - cx) * z_cm / fx
    xc = (vc - cy) * z_cm / fy
    pc = np.array([xc, yc, z_cm, 1.0], dtype=np.float64)

    pw = T @ pc
    pw[0] = -1 * pw[0] + OFFSET_X
    pw[1] = -1 * pw[1] + OFFSET_Y
    pw[2] = -1 * pw[2] + OFFSET_Z

    return float(pw[0]), float(pw[1]), float(pw[2])


def gemini_to_pixel(center_normalized: list, roi: tuple) -> tuple:
    """Gemini 정규화 좌표(0~1000) -> 전체 이미지 픽셀 좌표(u,v)."""
    cy, cx = center_normalized
    rx, ry, rw, rh = roi

    px_in_roi = int(cx / GEMINI_COORD_RANGE * rw)
    py_in_roi = int(cy / GEMINI_COORD_RANGE * rh)

    u = rx + px_in_roi
    v = ry + py_in_roi
    return u, v


def gemini_to_robot(center_normalized: list, roi: tuple, depth_map_m: np.ndarray):
    """Gemini center -> 로봇 좌표(tx,ty,tz). 실패 시 None."""
    u, v = gemini_to_pixel(center_normalized, roi)
    result = pixel_to_robot(u, v, depth_map_m)
    if result is None:
        print(f"[경고] 좌표 변환 실패 (u={u}, v={v})")
        return None

    tx, ty, tz = result
    print(f"[좌표] Gemini{center_normalized} -> pixel({u},{v}) -> robot({tx:.1f}, {ty:.1f}, {tz:.1f}) cm")
    return tx, ty, tz


# Legacy API compatibility (for older scripts)
def load_transform_matrix(filepath: str = None) -> np.ndarray:
    """기존 테스트 스크립트 호환용: 3x3 identity 또는 파일 로드."""
    if filepath:
        try:
            matrix = np.load(filepath)
            if matrix.shape in ((3, 3), (4, 4)):
                return matrix
        except Exception:
            pass
    return np.eye(3)


def uv_to_robot_coords(center_normalized: list, roi: tuple, transform_matrix: np.ndarray) -> tuple:
    """기존 테스트 스크립트 호환용 2D 변환."""
    u, v = gemini_to_pixel(center_normalized, roi)
    uv_h = np.array([u, v, 1.0], dtype=np.float64)
    robot_h = transform_matrix @ uv_h
    tx = float(robot_h[0] / robot_h[2])
    ty = float(robot_h[1] / robot_h[2])
    print(f"[좌표-legacy] Gemini{center_normalized} -> px({u},{v}) -> robot({tx:.2f},{ty:.2f})")
    return tx, ty
