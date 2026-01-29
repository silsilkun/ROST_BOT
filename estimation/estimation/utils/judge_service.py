# judge_service.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from estimation.utils.judge_core import JudgeCore


class EstimationJudge(Node):
    def __init__(self):
        super().__init__("estimation_judge")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("coords_topic", "/perception/waste_coordinates"),
                ("image_topic_raw", "/perception/waste_image_raw"),
                ("image_topic_vis", "/perception/waste_image_vis"),
                ("output_topic", "/estimation/type_id"),
                ("unknown_type_id", -1.0),
                ("max_age_sec", 1.0),
                ("drop_if_busy", True),
                ("jpeg_quality", 90),
            ],
        )

        coords_topic = self.get_parameter("coords_topic").value
        image_topic_raw = self.get_parameter("image_topic_raw").value
        image_topic_vis = self.get_parameter("image_topic_vis").value
        output_topic = self.get_parameter("output_topic").value
        unknown_type_id = self.get_parameter("unknown_type_id").value
        max_age_sec = self.get_parameter("max_age_sec").value
        drop_if_busy = self.get_parameter("drop_if_busy").value
        jpeg_quality = self.get_parameter("jpeg_quality").value

        pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.core = JudgeCore(
            node=self,
            pub=pub,
            unknown_type_id=unknown_type_id,
            max_age_sec=max_age_sec,
            drop_if_busy=drop_if_busy,
            jpeg_quality=jpeg_quality,
        )

        self.create_subscription(Float32MultiArray, coords_topic, self.core.on_coords, 10)
        self.create_subscription(Image, image_topic_raw, self.core.on_image_raw, 10)
        self.create_subscription(Image, image_topic_vis, self.core.on_image_vis, 10)

        self.get_logger().info(
            f"[Judge] Ready. coords={coords_topic} raw={image_topic_raw} "
            f"vis={image_topic_vis} -> {output_topic}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EstimationJudge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.shutdown()
        node.destroy_node()
        rclpy.shutdown()
