import cv2
import numpy as np
from coordinatetr import Coordinate

# ===============================
# 전역 상태
# ===============================
depth_frame_global = None
color_image_global = None
current_points = []

# ===============================
# 프레임 업데이트
# ===============================
def update_depth_frame(frame):
    global depth_frame_global
    depth_frame_global = frame

def update_color_image(frame):
    global color_image_global
    color_image_global = frame

# ===============================
# 마우스 클릭 콜백
# ===============================
def mouse_callback(event, x, y, flags, param):
    global current_points, depth_frame_global
    if event == cv2.EVENT_LBUTTONDOWN:
        if depth_frame_global is None:
            print("Depth 프레임 없음")
            return
        depth_mm = depth_frame_global.get_distance(x, y) * 1000.0
        current_points.append((x, y, depth_mm))
        print(f"Click: x={x}, y={y}, depth={depth_mm:.1f} mm")

# ===============================
# 저장 / 리셋 관련
# ===============================
def Save_Cam():
    """
    - 클릭한 좌표를 3D world 좌표로 변환
    - color 이미지 / depth 이미지 반환
    """
    global depth_frame_global, color_image_global
    if depth_frame_global is None or color_image_global is None:
        print("프레임 없음")
        return None, None, []

    color_image = color_image_global.copy()
    depth_image = np.asanyarray(depth_frame_global.get_data())

    picker = Coordinate()
    points_3d = []

    for u, v, _ in current_points:
        Pw = picker.pixel_to_world(u, v, depth_frame_global)
        if Pw is not None:
            points_3d.append(Pw[:3])

    if points_3d:
        print(f"3D 좌표 계산 완료 ({len(points_3d)}개 점)")
    else:
        print("저장할 3D 좌표 없음")

    return color_image, depth_image, points_3d

def reset_points():
    global current_points
    current_points.clear()
    print("포인트 리셋 완료")

def get_saved_points():
    return current_points
