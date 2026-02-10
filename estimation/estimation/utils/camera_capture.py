"""
R.O.S.T - 카메라 캡처 (camera_capture.py)
RealSense RGB 스냅샷 캡처 + ROI/bbox 크롭
"""

import numpy as np
import cv2
from config import REALSENSE_WIDTH, REALSENSE_HEIGHT, REALSENSE_FPS, GEMINI_COORD_RANGE

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("[경고] pyrealsense2 없음 → 테스트 모드")


def init_camera():
    """RealSense 카메라 초기화. 없으면 None(테스트 모드)."""
    if not REALSENSE_AVAILABLE:
        return None

    pipeline = rs.pipeline()
    config = rs.config()
    # [수정 포인트] 스트림 설정을 바꾸려면 여기만 수정
    config.enable_stream(rs.stream.color, REALSENSE_WIDTH, REALSENSE_HEIGHT,
                         rs.format.bgr8, REALSENSE_FPS)
    pipeline.start(config)
    # 오토 노출 안정화 (첫 30프레임 버리기)
    for _ in range(30):
        pipeline.wait_for_frames()
    print(f"[카메라] 초기화 완료 ({REALSENSE_WIDTH}x{REALSENSE_HEIGHT})")
    return pipeline


def stop_camera(pipeline):
    """카메라 종료"""
    if pipeline is not None:
        pipeline.stop()


def capture_snapshot(pipeline) -> np.ndarray:
    """
    RGB 사진 1장 캡처.
    pipeline이 None이면 더미 이미지 반환 (테스트용).
    """
    if pipeline is None:
        return np.zeros((REALSENSE_HEIGHT, REALSENSE_WIDTH, 3), dtype=np.uint8)

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    # [안전장치] 프레임 획득 실패
    if not color_frame:
        raise RuntimeError("[에러] RGB 프레임을 가져올 수 없습니다.")
    return np.asanyarray(color_frame.get_data())


def crop_to_roi(frame: np.ndarray, roi: tuple) -> np.ndarray:
    """
    전체 프레임에서 ROI만 잘라낸다.
    frame: 전체 이미지, roi: (x, y, w, h)
    """
    x, y, w, h = roi
    # [안전장치] ROI가 프레임 범위를 벗어나지 않는지 확인
    fh, fw = frame.shape[:2]
    x, y = max(0, x), max(0, y)
    w = min(w, fw - x)
    h = min(h, fh - y)
    return frame[y:y+h, x:x+w]


def crop_to_bbox(roi_image: np.ndarray, bbox_normalized: list,
                 margin_ratio: float = 0.1) -> np.ndarray:
    """
    Gemini가 리턴한 bbox 영역을 ROI 이미지에서 잘라낸다.
    확대해서 보내야 분류 정확도가 올라간다.

    bbox_normalized: [ymin, xmin, ymax, xmax] (0~1000)
    margin_ratio: 여유 마진 비율 (기본 10%)
    """
    h, w = roi_image.shape[:2]
    ymin, xmin, ymax, xmax = bbox_normalized

    # [수정 포인트] 정규화 범위가 바뀌면 GEMINI_COORD_RANGE만 수정
    px = lambda val, size: int(val / GEMINI_COORD_RANGE * size)
    x1, y1 = px(xmin, w), px(ymin, h)
    x2, y2 = px(xmax, w), px(ymax, h)

    # 마진 추가 (너무 타이트하면 맥락 소실)
    mx = int((x2 - x1) * margin_ratio)
    my = int((y2 - y1) * margin_ratio)
    x1, y1 = max(0, x1 - mx), max(0, y1 - my)
    x2, y2 = min(w, x2 + mx), min(h, y2 + my)

    # [안전장치] 크롭 영역이 유효한지 확인
    if x2 <= x1 or y2 <= y1:
        print("[경고] bbox 크롭 영역이 유효하지 않음 → 원본 반환")
        return roi_image

    return roi_image[y1:y2, x1:x2]
