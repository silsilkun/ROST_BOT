# coordinate.py
import os
import numpy as np
import cv2

from .settings import CALIB_PATH


class Coordinate:
    def __init__(self):
        self.T_cam_to_work = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration()

    def load_calibration(self):
        if not os.path.exists(CALIB_PATH):
            raise FileNotFoundError(f" '{CALIB_PATH}' 파일이 없습니다.")
        data = np.load(CALIB_PATH)
        self.T_cam_to_work = data["T_cam_to_work"]
        self.camera_matrix = data["camera_matrix"]
        self.dist_coeffs = data["dist_coeffs"]

    def pixel_to_world(self, u, v, depth_frame):
        depth_list = []
        for i in range(-2, 3):
            for j in range(-2, 3):
                d = depth_frame.get_distance(u + i, v + j)
                if d > 0:
                    depth_list.append(d)

        if not depth_list:
            return None

        depth_cm = np.median(depth_list) * 100.0  # m -> cm

        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        # ===============================
        # 🔥 distortion 보정 추가된 부분
        # ===============================
        pts = np.array([[[u, v]]], dtype=np.float32)
        undist = cv2.undistortPoints(
            pts,
            self.camera_matrix,
            self.dist_coeffs,
            P=self.camera_matrix
        )
        u_corr, v_corr = undist[0, 0]
        # ===============================

        # pixel -> camera (기존 수식 그대로, 입력만 보정됨)
        Yc = (u_corr - cx) * depth_cm / fx
        Xc = (v_corr - cy) * depth_cm / fy
        Zc = depth_cm

        Pc = np.array([Xc, Yc, Zc, 1.0])
        Pw = self.T_cam_to_work @ Pc

        # 실환경 보정값 (하드코딩 유지)
        Pw[0] = -Pw[0] + 81.5
        Pw[1] = -Pw[1] + 15.9
        Pw[2] = -Pw[2] + 2

        return Pw
