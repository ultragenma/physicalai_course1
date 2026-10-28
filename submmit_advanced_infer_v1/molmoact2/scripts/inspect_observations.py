#!/usr/bin/env python3
"""Standalone ROS observation inspector, also runnable through ``python3 -``.

Registers sensor subscriptions only. It saves graph and message metadata,
never image pixels or joint-position values. ROS imports are deferred so
``--help`` and metadata tests work on a PC without ROS.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import time
from datetime import datetime, timezone

BEGIN = "WHITEBOARD_OBSERVATIONS_JSON_BEGIN"
END = "WHITEBOARD_OBSERVATIONS_JSON_END"
DEFAULT_CAMERAS = (
    "cam_left_head=/zed/zed_node/left/image_rect_color",
    "cam_left_wrist=/camera_left/camera_left/color/image_rect_raw",
    "cam_right_wrist=/camera_right/camera_right/color/image_rect_raw",
)


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def image_metadata(msg) -> dict:
    return {"frame_id": msg.header.frame_id, "stamp_ns": stamp_ns(msg),
            "encoding": msg.encoding, "width": int(msg.width), "height": int(msg.height),
            "step_bytes": int(msg.step), "data_bytes": len(msg.data), "pixels_saved": False}


def joint_metadata(msg) -> dict:
    return {"stamp_ns": stamp_ns(msg), "name_order": list(msg.name),
            "position_count": len(msg.position), "velocity_count": len(msg.velocity),
            "positions_finite": all(math.isfinite(v) for v in msg.position), "values_saved": False}


def parse_cameras(values) -> dict[str, str]:
    result = {}
    for value in values:
        alias, separator, topic = value.partition("=")
        if not separator or not alias or not topic.startswith("/") or alias in result:
            raise ValueError("--camera must be a unique NAME=/absolute/topic")
        result[alias] = topic
    return result


def capture(args) -> dict:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState

    cameras = parse_cameras(args.camera or DEFAULT_CAMERAS)
    started_at = datetime.now(timezone.utc).isoformat()
    rclpy.init()
    node = Node("wb_readonly_observation_inspector")
    streams = {name: {"topic": topic, "count": 0, "sample": None} for name, topic in cameras.items()}
    streams["joint_state"] = {"topic": args.joint_state_topic, "count": 0, "sample": None}
    timing = {}

    def callback(name, make_metadata):
        def receive(msg):
            now = time.monotonic()
            stream = streams[name]
            stream["count"] += 1
            stream["sample"] = make_metadata(msg)
            state = timing.setdefault(name, {"first": now})
            state["last"] = now
        return receive

    try:
        for name, topic in cameras.items():
            node.create_subscription(Image, topic, callback(name, image_metadata), qos_profile_sensor_data)
        node.create_subscription(JointState, args.joint_state_topic,
                                 callback("joint_state", joint_metadata), qos_profile_sensor_data)
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(stream["count"] >= args.samples for stream in streams.values()):
                break
        graph = [{"name": name, "types": types} for name, types in node.get_topic_names_and_types()]
        graph.sort(key=lambda item: item["name"])
        now, ros_now = time.monotonic(), node.get_clock().now().nanoseconds
        for name, stream in streams.items():
            sample = stream["sample"]
            stream["message_type"] = "sensor_msgs/msg/JointState" if name == "joint_state" else "sensor_msgs/msg/Image"
            if sample is not None:
                measured = timing[name]
                span = measured["last"] - measured["first"]
                stream["receipt_age_s"] = now - measured["last"]
                stream["header_age_s"] = (ros_now - sample["stamp_ns"]) / 1e9
                stream["measured_hz"] = (stream["count"] - 1) / span if span > 0 else None
            # Include the discovered type even when the requested type yielded no samples.
            stream["discovered_types"] = next((item["types"] for item in graph
                                               if item["name"] == stream["topic"]), [])
            stream["publisher_count"] = len(node.get_publishers_info_by_topic(stream["topic"]))
        return {"schema_version": 1, "source": "live_ros_observation_metadata",
                "captured_at_utc": started_at,
                "complete": all(stream["count"] >= args.samples for stream in streams.values()),
                "environment": {"python": sys.version.split()[0], "ros_distro": os.getenv("ROS_DISTRO", ""),
                    "rmw": os.getenv("RMW_IMPLEMENTATION", ""), "domain_id": os.getenv("ROS_DOMAIN_ID", ""),
                    "zenoh_override_present": bool(os.getenv("ZENOH_CONFIG_OVERRIDE"))},
                "module_presence": {name: importlib.util.find_spec(name) is not None for name in
                                    ("numpy", "cv_bridge", "message_filters", "zmq", "msgpack", "msgpack_numpy")},
                "graph": {"topics": graph, "nodes": sorted(node.get_node_names()),
                          "note": "Topics may include this inspector's subscriptions; check publisher_count and received samples."},
                "cameras": {name: streams[name] for name in cameras}, "joint_state": streams["joint_state"],
                "actuator_commands_sent": 0, "pixels_saved": False, "joint_values_saved": False}
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", action="append", help="NAME=/raw/image/topic; repeat for each camera")
    ap.add_argument("--joint-state-topic", default="/joint_states")
    ap.add_argument("--timeout", type=float, default=12)
    ap.add_argument("--samples", type=int, default=3)
    args = ap.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 60 or args.samples < 1:
        ap.error("timeout must be in (0, 60] and samples must be positive")
    try:
        parse_cameras(args.camera or DEFAULT_CAMERAS)
        result = capture(args)
    except (ImportError, ValueError) as exc:
        ap.error(str(exc))
    print(BEGIN)
    print(json.dumps(result, indent=2, allow_nan=False))
    print(END)
    return 0 if result["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
