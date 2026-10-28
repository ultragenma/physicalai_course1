"""Isolated ROS integration fixture. The HTTP backend here is not an AI model."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.test_molmoact2 import FixtureBackend, checkpoint_fixture, profile
from wb_eraser.robot_schema import CAMERAS, STATIONARY_JOINTS
from wb_eraser.molmoact2_server import InferenceService, handler_for


def main():
    if (os.environ.get("ROS_DOMAIN_ID") != "73" or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
        raise RuntimeError("fixture requires the isolated domain-73/FastDDS/localhost-only test environment")
    out = ROOT / "runs/molmoact2/ros_fixture"
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = Node("molmoact2_fixture_observations_not_robot")
    publishers = {name: node.create_publisher(Image, "/molmo_fixture/" + name, qos_profile_sensor_data) for name in CAMERAS}
    joint_pub = node.create_publisher(JointState, "/molmo_fixture/joints", qos_profile_sensor_data)
    positions = np.linspace(-0.2, 0.2, 19, dtype=np.float32)
    contract = {"profile": "sg2_stationary", "observation_metadata_ready": True, "actuation_enabled": False,
                "test_fixture_not_robot": True,
                "cameras": {name: {"topic": "/molmo_fixture/" + name, "frame_id": name,
                                   "width": 12, "height": 8, "encoding": "rgb8"} for name in CAMERAS},
                "joint_state_topic": "/molmo_fixture/joints",
                "timing": {"max_sample_age_s": .5, "max_stamp_skew_s": .1}}
    contract_path = out / "fixture_contract.json"
    contract_path.write_text(json.dumps(contract, indent=2) + "\n")
    summary = {"test_fixture_not_robot": True, "backend_is_ai_model": False, "profiles": [], "complete": False,
               "hardware_commands_sent": 0}
    try:
        for kind in ["droid_probe", "sg2_native"]:
            with tempfile.TemporaryDirectory() as tmp:
                selected = profile(kind)
                schema = checkpoint_fixture(Path(tmp), selected)
                backend = FixtureBackend(schema)
                server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(InferenceService(schema, backend)))
                worker = threading.Thread(target=server.serve_forever, daemon=True)
                worker.start()
                result_path = out / (kind + ".json")
                command = ["python3", "-m", "wb_eraser.molmoact2_ros", str(contract_path),
                           "--profile", str(ROOT / f"configs/molmoact2/{kind}.json"),
                           "--url", f"http://127.0.0.1:{server.server_port}", "--frames", "3", "--timeout", "20",
                           "--instruction", "fixture observation; not a real robot", "--out", str(result_path)]
                process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                deadline = time.monotonic() + 30
                try:
                    while process.poll() is None and time.monotonic() < deadline:
                        stamp = node.get_clock().now().to_msg()
                        for i, name in enumerate(CAMERAS):
                            image = Image()
                            image.header.stamp, image.header.frame_id = stamp, name
                            image.width, image.height, image.encoding, image.step = 12, 8, "rgb8", 36
                            image.data = np.full((8, 12, 3), i * 50, dtype=np.uint8).tobytes()
                            publishers[name].publish(image)
                        joints = JointState()
                        joints.header.stamp = stamp
                        joints.name, joints.position = list(STATIONARY_JOINTS), positions.astype(float).tolist()
                        joint_pub.publish(joints)
                        rclpy.spin_once(node, timeout_sec=0.01)
                        time.sleep(0.04)
                    if process.poll() is None:
                        process.terminate()
                        raise TimeoutError("ROS client fixture timed out")
                    log, _ = process.communicate(timeout=3)
                    (out / (kind + "_client.log")).write_text(log)
                    if process.returncode:
                        raise RuntimeError(log)
                    result = json.loads(result_path.read_text())
                    assert result["complete"] and len(result["results"]) == 3
                    expected = positions[8:16] if kind == "droid_probe" else positions
                    for row in result["results"]:
                        assert row["result_fresh"] and row["actuator_commands_sent"] == 0
                        assert row["eligible_for_actuation"] is False
                        np.testing.assert_allclose(row["actions"], np.tile(expected, (15, 1)))
                    summary["profiles"].append({"kind": kind, "received_chunks": 3,
                                                 "action_shape": [15, len(expected)], "complete": True})
                finally:
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=5)
                    server.shutdown()
                    server.server_close()
                    worker.join(timeout=3)
        summary["complete"] = True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
