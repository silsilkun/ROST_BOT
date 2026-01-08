import pyrealsense2 as rs
import numpy as np

class RealSenseManager:
    """
    RealSense 카메라를 관리하는 클래스
    - 컬러 + Depth 스트림 초기화
    - 프레임 읽기
    - 카메라 종료
    """

    def __init__(self, width=1280, height=720, fps=30):
        """
        카메라 초기화 및 스트림 시작
        :param width: 영상 가로 해상도
        :param height: 영상 세로 해상도
        :param fps: 초당 프레임 수
        """
        # RealSense 파이프라인 생성
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 컬러 스트림 설정: BGR8 포맷
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        # Depth 스트림 설정: Z16 포맷
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        # 스트림 시작
        self.pipeline.start(self.config)
        print(f"RealSense 카메라 시작: {width}x{height} @ {fps}fps")

    def get_frames(self):
        """
        최신 컬러 + Depth 프레임 가져오기
        :return: color_image (numpy array), depth_frame (pyrealsense2 depth frame)
                 프레임이 없으면 (None, None) 반환
        """
        frames = self.pipeline.wait_for_frames()  # 최신 프레임 대기
        color_frame = frames.get_color_frame()    # 컬러 프레임 추출
        depth_frame = frames.get_depth_frame()    # Depth 프레임 추출

        # 프레임이 없으면 None 반환
        if not color_frame or not depth_frame:
            return None, None

        # 컬러 프레임을 numpy array로 변환 (OpenCV에서 사용 가능)
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_frame

    def stop(self):
        """
        카메라 스트림 종료
        """
        self.pipeline.stop()
        print("RealSense 카메라 종료")
