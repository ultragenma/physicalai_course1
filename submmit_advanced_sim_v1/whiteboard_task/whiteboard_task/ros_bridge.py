"""ROS 2 observation bridge with deliberately explicit integration boundaries.

It can subscribe to a configured RGB topic and archive image timestamps for
evaluation. It does NOT command motion until a reviewed AIWorker-specific
MoveIt/controller adapter is installed. This prevents sending guessed commands.
"""
from pathlib import Path
import math
import re
import time


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise SystemExit("This node requires a sourced ROS 2 environment (rclpy, sensor_msgs).") from exc

    class ObservationBridge(Node):
        def __init__(self):
            super().__init__("whiteboard_observation_bridge")
            self.declare_parameter("rgb_image_topic", "")
            self.declare_parameter("run_id", "")
            self.declare_parameter("output_directory", "")
            self.declare_parameter("metadata_period_s", 1.0)
            self.declare_parameter("max_metadata_records", 300)
            topic = self.get_parameter("rgb_image_topic").value
            if not topic:
                raise RuntimeError("rgb_image_topic is empty. Set it from verified AIWorker ROS 2 topics.")
            run_id = self.get_parameter("run_id").value
            if not isinstance(run_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", run_id):
                raise ValueError("run_id must be 1-64 letters, digits, underscores, or hyphens")
            self.metadata_period_s = float(self.get_parameter("metadata_period_s").value)
            self.max_metadata_records = int(self.get_parameter("max_metadata_records").value)
            if not math.isfinite(self.metadata_period_s) or self.metadata_period_s <= 0 or self.max_metadata_records <= 0:
                raise ValueError("metadata_period_s and max_metadata_records must be positive")
            self.last_saved_monotonic = 0.0
            self.saved_records = 0
            self.limit_warning_sent = False
            run_root = (Path("artifacts") / run_id).resolve()
            configured_output = self.get_parameter("output_directory").value
            self.output_directory = (Path(configured_output) if configured_output else run_root / "observations").resolve()
            if self.output_directory == run_root or run_root not in self.output_directory.parents:
                raise ValueError("output_directory must stay below artifacts/<run_id>/")
            # Use a new session directory for every launch; never overwrite an earlier recording.
            self.output_directory.mkdir(parents=True, exist_ok=False)
            self.subscription = self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
            self.get_logger().info(f"Observing verified image topic: {topic}")

        def on_image(self, message: Image) -> None:
            # Archive only metadata here. Image conversion/calibration must be added
            # after the confirmed camera encoding and intrinsics are available.
            now = time.monotonic()
            if self.saved_records >= self.max_metadata_records:
                if not self.limit_warning_sent:
                    self.get_logger().warning("Metadata record limit reached; ignoring further images")
                    self.limit_warning_sent = True
                return
            if now - self.last_saved_monotonic < self.metadata_period_s:
                return
            stamp = message.header.stamp
            metadata = self.output_directory / f"image_{self.saved_records:04d}_{stamp.sec}_{stamp.nanosec}.txt"
            with metadata.open("x", encoding="utf-8") as stream:
                stream.write(
                    f"frame_id={message.header.frame_id}\nencoding={message.encoding}\n"
                    f"width={message.width}\nheight={message.height}\n"
                )
            self.last_saved_monotonic = now
            self.saved_records += 1

    rclpy.init()
    node = None
    try:
        node = ObservationBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
