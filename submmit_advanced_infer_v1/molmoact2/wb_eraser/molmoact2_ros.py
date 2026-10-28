"""Read synchronized AIWorker observations, infer remotely, and save predictions."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time

from .robot_schema import CAMERAS, check_timestamps
from .molmoact2_client import MolmoAct2Client
from .molmoact2_contract import Profile


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("contract", type=Path, help="live metadata contract from wb_eraser.inference_contract")
    ap.add_argument("--profile", type=Path, default=Path("configs/molmoact2/droid_probe.json"))
    ap.add_argument("--url", default="http://127.0.0.1:8210")
    ap.add_argument("--instruction", default="Erase all marks from the whiteboard.")
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=90.)
    ap.add_argument("--request-timeout", type=float, default=30.)
    ap.add_argument("--max-result-age", type=float, default=5.)
    ap.add_argument("--out", type=Path, default=Path("runs/molmoact2/ros_inference.json"))
    args = ap.parse_args(argv)
    contract = json.loads(args.contract.read_text())
    if (contract.get("observation_metadata_ready") is not True or contract.get("profile") != "sg2_stationary"
            or contract.get("actuation_enabled") is not False):
        ap.error("a captured live stationary SG2 metadata contract with actuation disabled is required")
    if args.frames < 1 or not 0 < args.timeout <= 3600 or not 0 < args.max_result_age <= 60:
        ap.error("invalid frame count or timing limits")
    timing = contract["timing"]
    for key in ("max_sample_age_s", "max_stamp_skew_s"):
        if not math.isfinite(timing[key]) or not 0 < timing[key] <= 1.:
            ap.error("invalid observation age/synchronization contract")
    profile = Profile.load(args.profile)
    client = MolmoAct2Client(profile, args.url, args.request_timeout)
    report = {"complete": False, "mode": "aiworker_molmoact2_inference_only", "results": [],
              "actuator_commands_sent": 0, "policy_success_measured": False, "profile_kind": profile.kind}
    # Handshake before opening ROS subscriptions or sending any observations.
    try:
        report["server"] = client.check_server()
    except Exception as exc:
        report["error"] = str(exc)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2) + "\n")
        print(report["error"])
        return 1

    import rclpy
    from cv_bridge import CvBridge
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState

    rclpy.init()
    node = Node("aiworker_molmoact2_observation_client")
    bridge = CvBridge()
    lock = threading.Lock()
    latest = None

    def receive(*messages):
        nonlocal latest
        with lock:
            latest = (messages, time.monotonic())

    subscriptions = [Subscriber(node, Image, contract["cameras"][name]["topic"], qos_profile=qos_profile_sensor_data)
                     for name in CAMERAS]
    subscriptions.append(Subscriber(node, JointState, contract["joint_state_topic"], qos_profile=qos_profile_sensor_data))
    sync = ApproximateTimeSynchronizer(subscriptions, queue_size=10, slop=timing["max_stamp_skew_s"])
    sync.registerCallback(receive)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    deadline = time.monotonic() + args.timeout
    used = None
    try:
        while rclpy.ok() and len(report["results"]) < args.frames and time.monotonic() < deadline:
            with lock:
                packet = latest
            if packet is None or packet[1] == used:
                time.sleep(0.02)
                continue
            messages, received = packet
            used = received
            stamps = [int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec) for m in messages]
            check_timestamps(stamps, node.get_clock().now().nanoseconds, timing["max_sample_age_s"], timing["max_stamp_skew_s"])
            if time.monotonic() - received > timing["max_sample_age_s"]:
                raise ValueError("observation receipt is stale")
            images = {}
            for name, msg in zip(CAMERAS, messages[:3]):
                expected = contract["cameras"][name]
                if (msg.header.frame_id != expected["frame_id"] or msg.width != expected["width"]
                        or msg.height != expected["height"] or msg.encoding != expected["encoding"]):
                    raise ValueError("live camera metadata changed: " + name)
                images[name] = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            joints = messages[-1]
            actions, info = client.infer(images, list(joints.name), joints.position, args.instruction, stamps)
            age = time.monotonic() - received
            fresh = age <= args.max_result_age
            try:
                check_timestamps(stamps, node.get_clock().now().nanoseconds, args.max_result_age, timing["max_stamp_skew_s"])
            except ValueError:
                fresh = False
            report["results"].append({**info, "actions": actions.tolist(), "action_shape": list(actions.shape),
                                      "observation_age_s": age, "result_fresh": fresh,
                                      "eligible_for_actuation": False})
            print(f"MolmoAct2 returned {actions.shape}; fresh={fresh}; actuator commands=0", flush=True)
            if not fresh:
                raise ValueError("inference response is stale; prediction saved for inspection only")
        report["complete"] = len(report["results"]) == args.frames
        if not report["complete"]:
            report["error"] = "timed out waiting for synchronized ROS observations"
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
