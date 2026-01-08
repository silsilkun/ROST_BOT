import numpy as np
import pyrealsense2 as rs
import os

SAVE_FILE = "camcalib.npz"

class Coordinate:
    def __init__(self):
        # cam → work 변환행렬 & 카메라 파라미터
        self.T_cam_to_work = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration()

    def load_calibration(self):
        """캘리브레이션 로드"""
        if not os.path.exists(SAVE_FILE):
            raise FileNotFoundError(f" '{SAVE_FILE}' 파일이 없습니다.")
        data = np.load(SAVE_FILE)
        self.T_cam_to_work = data["T_cam_to_work"]
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs = data["dist_coeffs"]

    def pixel_to_world(self, u, v, depth_frame):
        """pixel + depth → world"""

        # 5×5 주변 depth 중앙값 (노이즈 완화)
        depth_list = []
        for i in range(-2, 3):
            for j in range(-2, 3):
                d = depth_frame.get_distance(u + i, v + j)
                if d > 0:
                    depth_list.append(d)

        if not depth_list:
            return None

        depth_cm = np.median(depth_list) * 100.0  # m → cm

        # 카메라 내부 파라미터
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        # pixel → camera 좌표계
        Yc = (u - cx) * depth_cm / fx
        Xc = (v - cy) * depth_cm / fy
        Zc = depth_cm

        # camera → world (동차좌표)
        Pc = np.array([Xc, Yc, Zc, 1.0])
        Pw = self.T_cam_to_work @ Pc

        # 실환경 보정값 (하드코딩 유지)
        Pw[0] = -Pw[0] + 81
        Pw[1] =  Pw[1] - 21
        Pw[2] = -Pw[2] + 1.5

        return Pw
