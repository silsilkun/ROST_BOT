import cv2
import numpy as np

# 전역 변수
depth_frame_global = None
color_image_global = None
current_points = []


# 마우스 클릭 콜백
def mouse_callback(event, x, y, flags, param):
    global depth_frame_global, current_points
    if event == cv2.EVENT_LBUTTONDOWN:
        if depth_frame_global is None:
            print("Depth 프레임 없음")
            return
        depth_mm = depth_frame_global.get_distance(x, y) * 1000.0
        current_points.append((x, y, depth_mm))
        print(f"Click: x={x}, y={y}, depth={depth_mm:.1f} mm")


# 콜백 설정
def setup_click_collector(window_name):
    cv2.setMouseCallback(window_name, mouse_callback)


# Depth 프레임 업데이트
def update_depth_frame(depth_frame):
    global depth_frame_global
    depth_frame_global = depth_frame


# Color 프레임 업데이트
def update_color_image(color_image):
    global color_image_global
    color_image_global = color_image


# s 키 기능: RGB / Depth 저장
def Save_Cam():
    global depth_frame_global, color_image_global

    if depth_frame_global is None or color_image_global is None:
        print("프레임 없음")
        return

    depth_image = np.asanyarray(depth_frame_global.get_data())

    cv2.imwrite("color.jpg", color_image_global)
    np.save("depth.npy", depth_image)

    print("color.jpg, depth.npy 저장 완료")

# 클릭 포인트 리셋
def reset_points():
    global current_points
    current_points.clear()
    print("포인트 리셋 완료")


# 클릭 좌표 반환
def get_saved_points():
    global current_points
    return current_points

