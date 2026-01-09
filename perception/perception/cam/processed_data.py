import os
import datetime
import numpy as np
import cv2
import cam_event  # Save_Cam 있는 파일

SAVE_FOLDER = "saved_data"
os.makedirs(SAVE_FOLDER, exist_ok=True)

color_image = None
depth_image = None
points_3d = []

def save_cam():
    """
    Save_Cam 호출 + 파일 저장 + 모듈 변수 갱신
    """
    global color_image, depth_image, points_3d

    color, depth, points = cam_event.Save_Cam()
    color_image = color
    depth_image = depth
    points_3d = points

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    if color is not None:
        cv2.imwrite(os.path.join(SAVE_FOLDER, f"color_{timestamp}.png"), color)
    if depth is not None:
        np.save(os.path.join(SAVE_FOLDER, f"depth_{timestamp}.npy"), depth)
    if points:
        print(points)

    print(f"스페이스바: color/depth/points 저장 완료")
    return color, depth, points
