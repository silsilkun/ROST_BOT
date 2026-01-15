import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import DR_init
from control.nodes.per_est_server import PerEstServer
from control.recycle import Recycle, ROBOT_ID
from estimation.nodes.add_node import EstimationAddNode
from estimation.nodes.judge_node import EstimationJudgeNode
from perception.utils import click_points, pipeline, realsense_loop
from rost_interfaces.srv import PerEstToControl


def _run_perception_loop(start_event: threading.Event, stop_event: threading.Event) -> None:
    def _on_confirm() -> None:
        if pipeline.processed_result.get("color") is None:
            pipeline.save_cam()
        start_event.set()

    realsense_loop.run(
        on_save=pipeline.save_cam,
        on_reset=click_points.reset_points,
        on_click=click_points.mouse_callback,
        on_confirm=_on_confirm,
        update_depth_frame=click_points.update_depth_frame,
        update_color_image=click_points.update_color_image,
        get_points=click_points.get_saved_points,
    )
    if pipeline.processed_result.get("color") is None:
        pipeline.save_cam()
    stop_event.set()


class PerceptionPublisher(Node):
    def __init__(self, start_event: threading.Event):
        super().__init__('perception_publisher')
        self._start_event = start_event
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._coords_pub = self.create_publisher(
            Float32MultiArray, '/perception/waste_coordinates', qos
        )
        self._image_raw_pub = self.create_publisher(
            Image, '/perception/waste_image_raw', qos
        )
        self._image_vis_pub = self.create_publisher(
            Image, '/perception/waste_image_vis', qos
        )
        self._bridge = CvBridge()
        self._stage = "coords"
        self._missing_logged = False
        self._waiting_logged = False
        self._timer = self.create_timer(0.5, self._try_publish)

    def _try_publish(self) -> None:
        if self._stage == "done":
            return
        if not self._start_event.is_set():
            if not self._waiting_logged:
                self.get_logger().info("waiting for enter to start estimation publish")
                self._waiting_logged = True
            return
        coords = pipeline.processed_result.get("flat_world_list")
        color = pipeline.processed_result.get("color")
        vis = pipeline.processed_result.get("vis")
        if coords is None or color is None:
            if not self._missing_logged:
                self.get_logger().warn("perception results missing; cannot publish")
                self._missing_logged = True
            return

        if self._stage == "coords":
            coords_msg = Float32MultiArray()
            coords_msg.data = [float(v) for v in coords]
            self._coords_pub.publish(coords_msg)
            self.get_logger().info(
                f"published perception coords_len={len(coords)}"
            )
            self._stage = "image"
            return

        raw_msg = self._bridge.cv2_to_imgmsg(color, encoding="bgr8")
        self._image_raw_pub.publish(raw_msg)
        if vis is None:
            if not self._missing_logged:
                self.get_logger().warn("vis image missing; publishing raw only")
                self._missing_logged = True
        else:
            vis_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self._image_vis_pub.publish(vis_msg)
        self.get_logger().info("published perception images (raw/vis)")
        self._stage = "done"
        self._timer.cancel()


class ControlRunner(Node):
    def __init__(self, recycle_node: Recycle, done_event):
        super().__init__('control_runner')
        self._recycle = recycle_node
        self._client = self.create_client(PerEstToControl, 'per_est_to_control')
        self._done_event = done_event
        self._pending = False
        self._empty_logged = False
        self._result = None
        self._timer = self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        if self._pending:
            return
        if not self._client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting again...')
            return
        self._pending = True
        future = self._client.call_async(PerEstToControl.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        self._pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'service call failed: {exc}')
            return

        if not response.success:
            self.get_logger().error(f'service returned failure: {response.message}')
            return

        trash_flat = [
            value for item in response.trash_list for value in list(item.data)
        ]
        bin_list = [list(item.data) for item in response.bin_list]
        if not trash_flat or not bin_list:
            if not self._empty_logged:
                self.get_logger().warn('empty trash/bin list received')
                self._empty_logged = True
            return

        self._empty_logged = False
        trash_list = [
            trash_flat[i:i + 5] for i in range(0, len(trash_flat), 5)
        ]
        self.get_logger().info(
            f'received trash_items={len(trash_list)} bins={len(bin_list)}'
        )
        self._timer.cancel()
        self._result = (trash_list, bin_list)
        self._done_event.set()

    def pop_result(self):
        return self._result


def main(args=None) -> None:
    rclpy.init(args=args)

    start_event = threading.Event()
    stop_event = threading.Event()
    done_event = threading.Event()

    dsr_node = rclpy.create_node('dsr_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    recycle_node = Recycle()
    server_node = PerEstServer()
    add_node = EstimationAddNode()
    judge_node = EstimationJudgeNode()
    perception_pub = PerceptionPublisher(start_event)
    runner_node = ControlRunner(recycle_node, done_event)

    executor = MultiThreadedExecutor()
    executor.add_node(dsr_node)
    executor.add_node(recycle_node)
    executor.add_node(server_node)
    executor.add_node(add_node)
    executor.add_node(judge_node)
    executor.add_node(perception_pub)
    executor.add_node(runner_node)

    def _spin_executor() -> None:
        try:
            while rclpy.ok() and not done_event.is_set() and not stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            result = runner_node.pop_result()
            if result:
                trash_list, bin_list = result
                recycle_node.run(trash_list, bin_list)
            judge_node._executor.shutdown(wait=False)
            executor.shutdown()
            runner_node.destroy_node()
            perception_pub.destroy_node()
            judge_node.destroy_node()
            add_node.destroy_node()
            server_node.destroy_node()
            recycle_node.destroy_node()
            dsr_node.destroy_node()
            rclpy.shutdown()

    spin_thread = threading.Thread(target=_spin_executor)
    spin_thread.start()

    _run_perception_loop(start_event, stop_event)
    stop_event.set()
    spin_thread.join()
