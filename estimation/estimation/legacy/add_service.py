# add_service.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from estimation.utils.add_core import AddCore


class EstimationAdd(Node):
    def __init__(self):
        super().__init__("estimation_add")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("coords_topic", "/perception/waste_coordinates"),
                ("type_topic", "/estimation/type_id"),
                ("output_topic", "/estimation/pickup_commands"),
                ("unknown_type_id", -1.0),
                ("max_age_sec", 20.0),
                ("sync_tolerance_sec", 1.0),
                ("allow_stale_coords", True),
                ("log_throttle_sec", 2.0),
            ],
        )

        coords_topic = self.get_parameter("coords_topic").value
        type_topic = self.get_parameter("type_topic").value
        output_topic = self.get_parameter("output_topic").value
        unknown_type_id = self.get_parameter("unknown_type_id").value
        max_age_sec = self.get_parameter("max_age_sec").value
        sync_tolerance_sec = self.get_parameter("sync_tolerance_sec").value
        allow_stale_coords = self.get_parameter("allow_stale_coords").value
        log_throttle_sec = self.get_parameter("log_throttle_sec").value

        pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.core = AddCore(
            node=self,
            pub=pub,
            unknown_type_id=unknown_type_id,
            max_age_sec=max_age_sec,
            sync_tolerance_sec=sync_tolerance_sec,
            allow_stale_coords=allow_stale_coords,
            log_throttle_sec=log_throttle_sec,
        )

        self.create_subscription(Float32MultiArray, coords_topic, self.core.on_coords, 10)
        self.create_subscription(Float32MultiArray, type_topic, self.core.on_types, 10)

        self.get_logger().info(
            f"[Add] Ready. coords={coords_topic}, types={type_topic} -> {output_topic}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EstimationAdd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
