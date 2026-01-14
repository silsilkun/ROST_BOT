import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class EstimationAddNode(Node):
    def __init__(self):
        super().__init__("estimation_add")

        # Integration: update these topic defaults to match perception/control wiring.
        self.declare_parameter("coords_topic", "/perception/xyz_list")
        self.declare_parameter("type_topic", "/estimation/type_id")
        self.declare_parameter("output_topic", "/estimation/xyz_type")
        self.declare_parameter("expected_count", 4)
        self.declare_parameter("max_age_sec", 2.0)
        # Integration: if control cannot handle -1, change to a safe fallback ID.
        self.declare_parameter("unknown_type_id", -1.0)

        self.coords_topic = self.get_parameter("coords_topic").get_parameter_value().string_value
        self.type_topic = self.get_parameter("type_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.expected_count = int(self.get_parameter("expected_count").get_parameter_value().integer_value)
        self.max_age_sec = float(self.get_parameter("max_age_sec").get_parameter_value().double_value)
        self.unknown_type_id = float(self.get_parameter("unknown_type_id").get_parameter_value().double_value)

        self._last_coords = None
        self._last_types = None
        self._last_coords_time = None
        self._last_types_time = None

        self._coords_sub = self.create_subscription(
            Float32MultiArray,
            self.coords_topic,
            self._on_coords,
            10,
        )
        self._types_sub = self.create_subscription(
            Float32MultiArray,
            self.type_topic,
            self._on_types,
            10,
        )
        self._publisher = self.create_publisher(Float32MultiArray, self.output_topic, 10)

        self.get_logger().info(
            f"EstimationAddNode listening on {self.coords_topic} and {self.type_topic}; "
            f"publishing to {self.output_topic}"
        )

    def _on_coords(self, msg: Float32MultiArray) -> None:
        self._last_coords = list(msg.data)
        self._last_coords_time = self.get_clock().now()
        self._try_publish()

    def _on_types(self, msg: Float32MultiArray) -> None:
        self._last_types = list(msg.data)
        self._last_types_time = self.get_clock().now()
        self._try_publish()

    def _try_publish(self) -> None:
        if self._last_coords is None or self._last_types is None:
            return

        # Integration: perception is assumed to send [x1,y1,z1, ...] in order.
        expected_coords_len = self.expected_count * 3
        if len(self._last_coords) != expected_coords_len:
            self.get_logger().warn(
                "coords length mismatch: "
                f"expected {expected_coords_len}, got {len(self._last_coords)}"
            )
            return

        # Integration: type list order must match perception coordinate order.
        if len(self._last_types) != self.expected_count:
            self.get_logger().warn(
                "type list length mismatch: "
                f"expected {self.expected_count}, got {len(self._last_types)}"
            )
            return

        if not self._is_fresh():
            self.get_logger().warn("skipping publish due to stale input")
            return

        packed = []
        for i in range(self.expected_count):
            x, y, z = self._last_coords[i * 3 : i * 3 + 3]
            t = self._last_types[i]
            if t is None:
                t = self.unknown_type_id
            packed.extend([x, y, z, t])

        out_msg = Float32MultiArray()
        out_msg.data = packed
        self._publisher.publish(out_msg)

    def _is_fresh(self) -> bool:
        now = self.get_clock().now()
        if self._last_coords_time is None or self._last_types_time is None:
            return False
        coords_age = (now - self._last_coords_time).nanoseconds / 1e9
        types_age = (now - self._last_types_time).nanoseconds / 1e9
        return coords_age <= self.max_age_sec and types_age <= self.max_age_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimationAddNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
