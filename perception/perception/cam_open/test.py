import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class ImageTest(Node):
    def __init__(self):
        super().__init__('image_test_node')
        # 구독 설정
        self.color_sub = self.create_subscription(Image, 'realsense_color_topic', self.color_cb, 10)
        self.depth_sub = self.create_subscription(Image, 'realsense_depth_topic', self.depth_cb, 10)
        
        self.c_img = None
        self.d_img = None

    def color_cb(self, msg):
        # 만약 보내는 쪽에서 이미 BGR로 보내고 있다면, cvtColor 없이 바로 reshape만 합니다.
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.c_img = img  # 색상 변환 없이 그대로 수신

    def depth_cb(self, msg):
        # Depth 데이터 시각화
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        self.d_img = cv2.applyColorMap(cv2.convertScaleAbs(raw, alpha=0.03), cv2.COLORMAP_JET)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.c_img is not None:
                cv2.imshow("Color View", self.c_img)
            if self.d_img is not None:
                cv2.imshow("Depth View", self.d_img)
                
            if cv2.waitKey(1) & 0xFF == 27:  # ESC 종료
                break
        
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = ImageTest()
    print("dasda")
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()