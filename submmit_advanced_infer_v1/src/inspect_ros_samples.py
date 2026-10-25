#!/usr/bin/env python3
"""Read camera metadata and JointState names without saving sensor payloads."""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


class SampleInspector(Node):
    def __init__(self, camera_topic: str, joint_state_topic: str) -> None:
        super().__init__("aiworker_readonly_sample_inspector")
        self.camera: dict[str, object] | None = None
        self.joint_state: dict[str, object] | None = None
        self._camera_sub = self.create_subscription(
            Image, camera_topic, self._on_camera, qos_profile_sensor_data,
        )
        self._joint_state_sub = self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state, qos_profile_sensor_data,
        )

    def _on_camera(self, msg: Image) -> None:
        if self.camera is None:
            self.camera = {
                "frame_id": msg.header.frame_id,
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "encoding": msg.encoding,
                "width": int(msg.width),
                "height": int(msg.height),
                "step_bytes": int(msg.step),
                "is_bigendian": int(msg.is_bigendian),
                "data_bytes": len(msg.data),
                "pixels_saved": False,
            }

    def _on_joint_state(self, msg: JointState) -> None:
        if self.joint_state is None:
            self.joint_state = {
                "stamp_sec": int(msg.header.stamp.sec),
                "stamp_nanosec": int(msg.header.stamp.nanosec),
                "name_order": list(msg.name),
                "position_count": len(msg.position),
                "velocity_count": len(msg.velocity),
                "effort_count": len(msg.effort),
            }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-topic", required=True)
    parser.add_argument("--joint-state-topic", required=True)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")

    rclpy.init()
    node = SampleInspector(args.camera_topic, args.joint_state_topic)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if node.camera is not None and node.joint_state is not None:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        payload = {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "camera_topic": args.camera_topic,
            "camera": node.camera,
            "joint_state_topic": args.joint_state_topic,
            "joint_state": node.joint_state,
            "warning": "Contains internal topic and joint names; redact before public sharing.",
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if payload["camera"] is not None and payload["joint_state"] is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
