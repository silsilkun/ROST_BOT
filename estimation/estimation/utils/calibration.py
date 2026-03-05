"""
R.O.S.T - 캘리브레이션 (calibration.py)
uv 좌표(이미지 픽셀) → 로봇 좌표(로봇팔이 실제 이동할 좌표) 변환
"""

import os
import numpy as np
import cv2
try:
    from estimation.utils import config as cfg
except Exception:
    import config as cfg

GEMINI_COORD_RANGE = cfg.GEMINI_COORD_RANGE

# ── 캘리브레이션/좌표 변환 파라미터 ─────────────────────
CALIB_NPZ = "camcalib.npz"
DEPTH_MIN_M = 0.20
DEPTH_MAX_M = 2.00
M_TO_CM = 100.0
DEPTH_SAMPLE_R = 2

_CALIB_CACHE = None


def _load_calib(filepath: str = None):
    global _CALIB_CACHE
    if _CALIB_CACHE is not None:
        return _CALIB_CACHE

    if filepath is None:
        base = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(base, CALIB_NPZ)

    if not os.path.isabs(filepath):
        base = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(base, filepath)

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"캘리브 파일 없음: {filepath}")

    data = np.load(filepath)
    if "T_cam_to_work" not in data or "camera_matrix" not in data or "dist_coeffs" not in data:
        raise KeyError(f"캘리브 파일 키 누락: {list(data.keys())}")

    _CALIB_CACHE = {
        "T": data["T_cam_to_work"].astype(np.float64),
        "K": data["camera_matrix"].astype(np.float64),
        "D": data["dist_coeffs"].astype(np.float64),
    }
    print(f"[캘리브레이션] camcalib 로드: {filepath} (T:{_CALIB_CACHE['T'].shape}, "
          f"K:{_CALIB_CACHE['K'].shape})")
    return _CALIB_CACHE


def load_transform_matrix(filepath: str = None):
    return _load_calib(filepath=filepath)


def gemini_to_pixel(center_normalized: list, roi: tuple) -> tuple:
    cy_norm, cx_norm = center_normalized
    roi_x, roi_y, roi_w, roi_h = roi

    #정규화 범위가 바뀌면 config.py의 GEMINI_COORD_RANGE만 수정
    local_x = int(cx_norm / GEMINI_COORD_RANGE * roi_w)
    local_y = int(cy_norm / GEMINI_COORD_RANGE * roi_h)

    # ROI offset 더하기 → 전체 이미지 기준
    pixel_u = roi_x + local_x
    pixel_v = roi_y + local_y
    return (pixel_u, pixel_v)


def pixel_to_robot(pixel_u: int, pixel_v: int, depth_map_m: np.ndarray,
                   transform_matrix):
    if isinstance(transform_matrix, dict):
        K = transform_matrix["K"]
        D = transform_matrix["D"]
        T = transform_matrix["T"]

        if depth_map_m is None:
            print("[경고] depth_map 없음 → 좌표 계산 실패")
            return None

        H, W = depth_map_m.shape[:2]
        u = int(np.clip(pixel_u, 0, W - 1))
        v = int(np.clip(pixel_v, 0, H - 1))

        depths = []
        for du in range(-DEPTH_SAMPLE_R, DEPTH_SAMPLE_R + 1):
            for dv in range(-DEPTH_SAMPLE_R, DEPTH_SAMPLE_R + 1):
                uu = u + du
                vv = v + dv
                if 0 <= uu < W and 0 <= vv < H:
                    d = float(depth_map_m[vv, uu])
                    if d > 0.0 and (DEPTH_MIN_M <= d <= DEPTH_MAX_M):
                        depths.append(d)

        if not depths:
            print("[경고] depth 샘플 없음 → 좌표 계산 실패")
            return None

        Z_cm = float(np.median(depths)) * M_TO_CM

        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])

        pts = np.array([[[u, v]]], dtype=np.float32)
        und = cv2.undistortPoints(pts, K, D, P=K)
        uc, vc = float(und[0, 0, 0]), float(und[0, 0, 1])

        # 규약: u->Yc, v->Xc
        Yc = (uc - cx) * Z_cm / fx
        Xc = (vc - cy) * Z_cm / fy
        Pc = np.array([Xc, Yc, Z_cm, 1.0], dtype=np.float64)

        Pw = T @ Pc

        # 규약 고정
        Pw[0] = -1 * Pw[0] + 81.5
        Pw[1] = -1 * Pw[1] + 15.9
        Pw[2] = -1 * Pw[2] + 0.0

        tx_mm = float(Pw[0] * 10.0)
        ty_mm = float(Pw[1] * 10.0)
        return (tx_mm, ty_mm)

    print("[경고] 지원하지 않는 변환 행렬 형식")
    return None


def uv_to_robot_coords(center_normalized: list, roi: tuple,
                        depth_map_m: np.ndarray, transform_matrix) -> tuple:
    pixel_u, pixel_v = gemini_to_pixel(center_normalized, roi)
    result = pixel_to_robot(pixel_u, pixel_v, depth_map_m, transform_matrix)
    if result is None:
        return None
    tx, ty = result
    print(f"[좌표] Gemini{center_normalized} → px({pixel_u},{pixel_v}) → robot({tx:.2f},{ty:.2f})")
    return (tx, ty)


def gemini_to_robot(center_normalized: list, roi: tuple, depth_map_m: np.ndarray):
    T = _load_calib()
    result = uv_to_robot_coords(center_normalized, roi, depth_map_m, T)
    if result is None:
        return None
    tx, ty = result
    return (tx, ty, 0.0)
