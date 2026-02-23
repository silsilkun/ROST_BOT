"""
R.O.S.T - 카메라 캡처 (camera_capture.py)
RealSense RGB + Depth 스냅샷 캡처 + ROI/bbox 크롭
"""

import numpy as np
from estimation.utils.config import (
    REALSENSE_WIDTH,
    REALSENSE_HEIGHT,
    REALSENSE_FPS,
    GEMINI_COORD_RANGE,
)

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("[경고] pyrealsense2 없음 -> 테스트 모드")


def init_camera():
    """
    RealSense 카메라 초기화 (RGB + Depth).
    Returns: (pipeline, align) 튜플
    """
    if not REALSENSE_AVAILABLE:
        return None, None

    # 요청 설정 우선, 실패 시 호환 프로파일로 폴백
    candidates = [
        (REALSENSE_WIDTH, REALSENSE_HEIGHT, REALSENSE_FPS),
        (640, 480, 15),
        (640, 480, 5),
    ]

    last_error = None
    for width, height, fps in candidates:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        try:
            pipeline.start(config)
            align = rs.align(rs.stream.color)
            for _ in range(30):
                pipeline.wait_for_frames()
            print(f"[카메라] 초기화 완료 ({width}x{height}@{fps}, RGB+Depth)")
            return pipeline, align
        except Exception as exc:
            last_error = exc
            try:
                pipeline.stop()
            except Exception:
                pass
            print(f"[경고] RealSense 프로파일 실패 ({width}x{height}@{fps}): {exc}")

    raise RuntimeError(f"[에러] 사용 가능한 RealSense 스트림 프로파일이 없습니다: {last_error}")


def stop_camera(cam):
    """카메라 종료. cam = (pipeline, align) 튜플"""
    if cam is None:
        return
    pipeline = cam[0] if isinstance(cam, tuple) else cam
    if pipeline is not None:
        pipeline.stop()


def capture_snapshot(cam) -> np.ndarray:
    """
    RGB 사진 1장 캡처.
    cam이 None이면 더미 이미지 반환.
    """
    if cam is None or cam[0] is None:
        return np.zeros((REALSENSE_HEIGHT, REALSENSE_WIDTH, 3), dtype=np.uint8)

    pipeline, align = cam
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()

    if not color_frame:
        raise RuntimeError("[에러] RGB 프레임을 가져올 수 없습니다.")
    return np.asanyarray(color_frame.get_data())


def capture_snapshot_and_depth(cam):
    """
    RGB + Depth 동시 캡처.
    Returns: (color_image, depth_m)
    """
    if cam is None or cam[0] is None:
        color = np.zeros((REALSENSE_HEIGHT, REALSENSE_WIDTH, 3), dtype=np.uint8)
        depth = np.full((REALSENSE_HEIGHT, REALSENSE_WIDTH), 0.25, dtype=np.float32)
        return color, depth

    pipeline, align = cam
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)

    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        raise RuntimeError("[에러] RGB/Depth 프레임을 가져올 수 없습니다.")

    color = np.asanyarray(color_frame.get_data())
    depth_raw = np.asanyarray(depth_frame.get_data())
    depth_m = depth_raw.astype(np.float32) * depth_frame.get_units()
    return color, depth_m


def crop_to_roi(frame: np.ndarray, roi: tuple) -> np.ndarray:
    """전체 프레임에서 ROI만 잘라낸다."""
    x, y, w, h = roi
    fh, fw = frame.shape[:2]
    x, y = max(0, x), max(0, y)
    w = min(w, fw - x)
    h = min(h, fh - y)
    return frame[y:y + h, x:x + w]


def crop_to_bbox(roi_image: np.ndarray, bbox_normalized: list, margin_ratio: float = 0.1) -> np.ndarray:
    """
    Gemini bbox를 ROI 이미지에서 잘라낸다.
    bbox_normalized: [ymin, xmin, ymax, xmax] (0~1000)
    """
    h, w = roi_image.shape[:2]
    ymin, xmin, ymax, xmax = bbox_normalized

    px = lambda val, size: int(val / GEMINI_COORD_RANGE * size)
    x1, y1 = px(xmin, w), px(ymin, h)
    x2, y2 = px(xmax, w), px(ymax, h)

    mx = int((x2 - x1) * margin_ratio)
    my = int((y2 - y1) * margin_ratio)
    x1, y1 = max(0, x1 - mx), max(0, y1 - my)
    x2, y2 = min(w, x2 + mx), min(h, y2 + my)

    if x2 <= x1 or y2 <= y1:
        print("[경고] bbox 크롭 영역이 유효하지 않음 -> 원본 반환")
        return roi_image

    return roi_image[y1:y2, x1:x2]
