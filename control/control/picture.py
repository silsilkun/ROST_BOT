# picture.py
import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR = os.path.join(BASE_DIR, "picture")

# Perception ROI (u1, v1, u2, v2)
ROI = (450, 160, 820, 440)


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _crop_roi(img: np.ndarray) -> np.ndarray:
    x1, y1, x2, y2 = ROI
    h, w = img.shape[:2]
    x1 = max(0, min(w - 1, x1))
    x2 = max(0, min(w, x2))
    y1 = max(0, min(h - 1, y1))
    y2 = max(0, min(h, y2))
    if x2 <= x1 or y2 <= y1:
        return img
    return img[y1:y2, x1:x2]


def main() -> None:
    _ensure_dir(SAVE_DIR)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
    except RuntimeError as exc:
        print(f"RealSense start failed ({exc}), fallback to 640x480")
        fallback = rs.config()
        fallback.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(fallback)

    print("스페이스바: ROI 캡처 저장 | esc: 종료")
    window_name = "RealSense Color"
    cv2.namedWindow(window_name)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            display_image = color_image.copy()

            # ROI 표시
            x1, y1, x2, y2 = ROI
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.imshow(window_name, display_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                cropped = _crop_roi(color_image)
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"roi_{ts}.jpg"
                path = os.path.join(SAVE_DIR, filename)
                ok = cv2.imwrite(path, cropped)
                if ok:
                    print(f"저장 완료: {path}")
                else:
                    print("저장 실패")
            elif key == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
