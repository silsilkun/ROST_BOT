# judge_node.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String
from dotenv import find_dotenv, load_dotenv

from .prompt import PromptConfig
from .utils import ImageLoader
from .logic import EstimationLogic  #logic 파일에서 가져오기

class EstimationJudgeNode(Node):
    def __init__(self):
        super().__init__("estimation_judge")
        load_dotenv(find_dotenv(usecwd=True))
        
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_mode", "compressed"), ("image_topic", "/perception/image_path"),
                ("compressed_topic", "/perception/image/compressed"), ("output_topic", "/estimation/type_id"),
                ("expected_count", 4), ("unknown_type_id", -1.0)
            ]
        )
        
        #Logic 인스턴스 생성
        self.loader = ImageLoader()
        self.logic = EstimationLogic(PromptConfig(), os.getenv("GEMINI_API_KEY"))

        if self.get_parameter("input_mode").value == "compressed":
            self.create_subscription(CompressedImage, self.get_parameter("compressed_topic").value, self._on_compressed, 10)
        else:
            self.create_subscription(String, self.get_parameter("image_topic").value, self._on_path, 10)
            
        self.pub = self.create_publisher(Float32MultiArray, self.get_parameter("output_topic").value, 10)
        self.get_logger().info("Judge Node Ready (Logic Loaded)!")

    def _on_path(self, msg):
        b = self.loader.read_file(msg.data)
        if b: self._execute(b, self.loader.guess_mime_type(msg.data))
        else: self.get_logger().warn(f"File not found: {msg.data}")

    def _on_compressed(self, msg):
        if msg.data: self._execute(bytes(msg.data), self.loader.guess_mime_type(msg.format))

    def _execute(self, b_data, mime):
        #self.logic 사용
        ids = self.logic.run_inference(
            b_data, mime, 
            self.get_parameter("expected_count").value, 
            self.get_parameter("unknown_type_id").value
        )
        self.pub.publish(Float32MultiArray(data=[float(x) for x in ids]))

def main(args=None):
    rclpy.init(args=args)
    node = EstimationJudgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()