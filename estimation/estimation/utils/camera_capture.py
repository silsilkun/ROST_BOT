"""
R.O.S.T - 카메라 캡처 (camera_capture.py)
RealSense RGB + Depth 스냅샷 캡처 + ROI/bbox 크롭

[변경] depth 스트림 추가 + align(RGB-Depth 정렬)
       → calibration에서 좌표 변환할 때 depth map 필요
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
    """
    RealSense 카메라 초기화 (RGB + Depth).
    없으면 None(테스트 모드).
    Returns: (pipeline, align) 튜플
    """
    if not REALSENSE_AVAILABLE:
        return None, None

    pipeline = rs.pipeline()
    config = rs.config()

    # [수정 포인트] 스트림 설정을 바꾸려면 여기만 수정
    config.enable_stream(rs.stream.color, REALSENSE_WIDTH, REALSENSE_HEIGHT,
                         rs.format.bgr8, REALSENSE_FPS)
    config.enable_stream(rs.stream.depth, REALSENSE_WIDTH, REALSENSE_HEIGHT,
                         rs.format.z16, REALSENSE_FPS)

    pipeline.start(config)

    # Depth를 RGB에 맞춰 정렬 (픽셀 위치가 1:1 대응되게)
    align = rs.align(rs.stream.color)

    # 오토 노출 안정화 (첫 30프레임 버리기)
    for _ in range(30):
        pipeline.wait_for_frames()

    print(f"[카메라] 초기화 완료 ({REALSENSE_WIDTH}x{REALSENSE_HEIGHT}, RGB+Depth)")
    return pipeline, align


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
    cam이 None이면 더미 이미지 반환 (테스트용).
    """
    if cam is None or cam[0] is None:
        return np.zeros((REALSENSE_HEIGHT, REALSENSE_WIDTH, 3), dtype=np.uint8)

    pipeline, align = cam
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()

    # [안전장치] 프레임 획득 실패
    if not color_frame:
        raise RuntimeError("[에러] RGB 프레임을 가져올 수 없습니다.")
    return np.asanyarray(color_frame.get_data())


def capture_snapshot_and_depth(cam):
    """
    RGB + Depth 동시 캡처 (같은 프레임에서).
    → RGB와 Depth의 시간 차이가 없어야 좌표가 정확.
    Returns: (color_image, depth_m)
             depth_m: (H, W) float 배열, 단위=미터
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

    # [안전장치]
    if not color_frame or not depth_frame:
        raise RuntimeError("[에러] RGB/Depth 프레임을 가져올 수 없습니다.")

    color = np.asanyarray(color_frame.get_data())
    depth_raw = np.asanyarray(depth_frame.get_data())  # uint16, mm
    depth_m = depth_raw.astype(np.float32) * depth_frame.get_units()

    return color, depth_m


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
    bbox_normalized: [ymin, xmin, ymax, xmax] (0~1000)
    margin_ratio: 여유 마진 비율 (기본 10%)
    """
    h, w = roi_image.shape[:2]
    ymin, xmin, ymax, xmax = bbox_normalized

    # [수정 포인트] 정규화 범위가 바뀌면 GEMINI_COORD_RANGE만 수정
    px = lambda val, size: int(val / GEMINI_COORD_RANGE * size)
    x1, y1 = px(xmin, w), px(ymin, h)
    x2, y2 = px(xmax, w), px(ymax, h)

    # 마진 추가
    mx = int((x2 - x1) * margin_ratio)
    my = int((y2 - y1) * margin_ratio)
    x1, y1 = max(0, x1 - mx), max(0, y1 - my)
    x2, y2 = min(w, x2 + mx), min(h, y2 + my)

    # [안전장치]
    if x2 <= x1 or y2 <= y1:
        print("[경고] bbox 크롭 영역이 유효하지 않음 → 원본 반환")
        return roi_image

    return roi_image[y1:y2, x1:x2]
