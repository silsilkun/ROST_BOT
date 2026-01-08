# node.py
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String
from dotenv import find_dotenv, load_dotenv

# 우리가 쪼갠 파일들 import (같은 폴더에 있어야 함)
from . import prompt
from . import utils
from .gemini_api import GeminiClient

class EstimationJudgeNode(Node):
    def __init__(self):
        super().__init__("estimation_judge")
        load_dotenv(find_dotenv(usecwd=True))

        # 파라미터 선언 및 로드
        self.declare_parameter("input_mode", "compressed")
        self.declare_parameter("image_topic", "/perception/image_path")
        self.declare_parameter("compressed_topic", "/perception/image/compressed")
        self.declare_parameter("output_topic", "/estimation/type_id")
        self.declare_parameter("expected_count", 4)
        self.declare_parameter("model_name", prompt.DEFAULT_MODEL)
        self.declare_parameter("api_key_env", "GEMINI_API_KEY")
        self.declare_parameter("unknown_type_id", -1.0)
        self.declare_parameter("log_response", False)

        self.input_mode = self.get_val("input_mode")
        self.output_topic = self.get_val("output_topic")
        self.expected_count = self.get_val("expected_count", int)
        self.unknown_type_id = self.get_val("unknown_type_id", float)
        self.log_response = self.get_val("log_response", bool)
        
        # Gemini 클라이언트 준비
        api_key = os.getenv(self.get_val("api_key_env"))
        self.client = GeminiClient(
            api_key=api_key,
            model_name=self.get_val("model_name"),
            timeout=prompt.DEFAULT_TIMEOUT,
            temp=prompt.DEFAULT_TEMP,
            max_tokens=prompt.DEFAULT_MAX_TOKENS
        )

        self._publisher = self.create_publisher(Float32MultiArray, self.output_topic, 10)
        self._setup_subscriber()
        self.get_logger().info(f"Judge Node Ready. Mode: {self.input_mode}")

    def get_val(self, key, type_cast=str):
        val = self.get_parameter(key).get_parameter_value()
        if type_cast == int: return val.integer_value
        if type_cast == float: return val.double_value
        if type_cast == bool: return val.bool_value
        return val.string_value

    def _setup_subscriber(self):
        if self.input_mode == "compressed":
            self._subscription = self.create_subscription(
                CompressedImage, self.get_val("compressed_topic"), self._on_compressed, 10
            )
        else:
            self._subscription = self.create_subscription(
                String, self.get_val("image_topic"), self._on_path, 10
            )

    def _on_path(self, msg):
        path = msg.data.strip()
        img_bytes = utils.read_image_file(path)
        if not img_bytes:
            self.get_logger().warn(f"Cannot read image: {path}")
            self._pub_result([self.unknown_type_id] * self.expected_count)
            return
        self._process_image(img_bytes, utils.guess_mime_type(path))

    def _on_compressed(self, msg):
        if not msg.data: return
        self._process_image(bytes(msg.data), utils.guess_mime_from_format(msg.format))

    def _process_image(self, img_bytes, mime_type):
        prompt_text = prompt.get_prompt(self.expected_count, prompt.ALLOWED_LABELS)
        
        try:
            resp_text = self.client.generate(prompt_text, img_bytes, mime_type)
            if self.log_response: self.get_logger().info(f"Gemini Resp: {resp_text[:100]}...")
        except Exception as e:
            self.get_logger().error(f"Gemini API Error: {e}")
            self._pub_result([self.unknown_type_id] * self.expected_count)
            return

        labels = utils.parse_labels_from_text(resp_text)
        type_ids = utils.convert_labels_to_ids(labels, prompt.LABEL_TO_ID, self.unknown_type_id)
        
        # 개수 맞추기 (Padding / Truncating)
        curr_len = len(type_ids)
        if curr_len < self.expected_count:
            type_ids.extend([self.unknown_type_id] * (self.expected_count - curr_len))
        elif curr_len > self.expected_count:
            type_ids = type_ids[:self.expected_count]
            
        self._pub_result(type_ids)

    def _pub_result(self, data):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in data]
        self._publisher.publish(msg)

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
