"""ROS RGB-D -> fixture VLM -> real MoveL message -> Cartesian follower test.

The board stays unchanged deliberately, so task failure and working execution
can be verified separately. This is not a trained policy, IK or contact model.
"""

from http.server import ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotis_interfaces.msg import MoveL
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.board_render import BoardRenderer, BoardSpec, default_intrinsics, head_camera_pose
from wb_eraser.vlm_server import SelectionService, handler_for


class FixtureVision:
    text_valid = True

    def generate(self, rgb, prompt):
        if not self.text_valid:
            return "Unable to select a region."
        candidates = json.loads(prompt.split("Candidates: ", 1)[1])
        item = min(candidates, key=lambda c: c["bbox"][2] * c["bbox"][3])
        return json.dumps({"target_ids": [item["id"]]})


class CartesianFixture(Node):
    def __init__(self):
        super().__init__("hybrid_cartesian_fixture_not_robot")
        self.bridge = CvBridge()
        self.position = np.array([.25, -.30, 1.05])
        self.start_position = self.position.copy()
        self.target = self.position.copy()
        self.started, self.duration = time.monotonic(), 1.
        self.ignore_goals = False
        self.goals = []
        self.create_subscription(MoveL, "/r_goal_move", self.on_goal, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/r_gripper_pose", 10)
        self.image_pub = self.create_publisher(Image, "/hybrid_fixture/rgb", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/hybrid_fixture/depth", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/hybrid_fixture/info", qos_profile_sensor_data)
        self.K = default_intrinsics(672, 376, 340.)
        T = head_camera_pose()
        renderer = BoardRenderer(BoardSpec(), "numbers")
        self.rgb, self.depth = renderer.render(T, self.K, 672, 376, depth_noise=0.)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "base_link", "hybrid_fixture_optical"
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = T[:3, 3].tolist()
        vec, _ = cv2.Rodrigues(T[:3, :3])
        angle = float(np.linalg.norm(vec))
        qxyz = vec.ravel() / angle * math.sin(angle / 2.)
        (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z) = qxyz.tolist()
        transform.transform.rotation.w = math.cos(angle / 2.)
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(transform)
        self.create_timer(.04, self.tick)

    def on_goal(self, goal):
        assert goal.pose.header.frame_id == "base_link"
        p = goal.pose.pose.position
        target = np.array([p.x, p.y, p.z])
        duration = goal.time_from_start.sec + goal.time_from_start.nanosec / 1e9
        assert np.isfinite(target).all() and duration > 0
        self.goals.append({"target": target.tolist(), "duration_s": duration})
        if not self.ignore_goals:
            self.start_position, self.target = self.position.copy(), target
            self.started, self.duration = time.monotonic(), duration

    def tick(self):
        t = min(1., (time.monotonic() - self.started) / self.duration)
        self.position = self.start_position + (self.target - self.start_position) * t
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "base_link", stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position.tolist()
        pose.pose.orientation.w = 1.
        self.pose_pub.publish(pose)
        image = self.bridge.cv2_to_imgmsg(self.rgb, encoding="bgr8")
        image.header.frame_id, image.header.stamp = "hybrid_fixture_optical", stamp
        self.image_pub.publish(image)
        depth = self.bridge.cv2_to_imgmsg(self.depth, encoding="32FC1")
        depth.header.frame_id, depth.header.stamp = "hybrid_fixture_optical", stamp
        self.depth_pub.publish(depth)
        info = CameraInfo()
        info.header.frame_id, info.header.stamp = "hybrid_fixture_optical", stamp
        info.width, info.height = 672, 376
        info.k = self.K.ravel().tolist()
        self.info_pub.publish(info)


def main():
    if (os.environ.get("ROS_DOMAIN_ID") != "73" or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
        raise RuntimeError("this fixture requires the isolated localhost-only domain-73 container")
    out = ROOT / "runs/hybrid/ros_fixture"
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    scene = CartesianFixture()
    backend = FixtureVision()
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(SelectionService(
        backend, {"backend": "fixture_not_ai", "is_ai_model": False})))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    report = {"fixture_not_real_robot_or_ai": True, "contact_or_ik_simulated": False,
              "vendor_movel_fields": MoveL.get_fields_and_field_types(), "cases": [], "complete": False}
    try:
        for name, execute, valid, ignore in [
                ("plan_only", False, True, False),
                ("motion_complete_task_failed", True, True, False),
                ("invalid_model_reply", False, False, False),
                ("explicit_fallback_plan", False, False, False),
                ("stale_selection", False, False, False),
                ("controller_does_not_reach", True, True, True)]:
            backend.text_valid, scene.ignore_goals = valid, ignore
            scene.goals.clear()
            case_out = out / name
            command = ["python3", "-m", "wb_eraser.hybrid_run", "--rounds", "1", "--no-head",
                       "--image-topic", "/hybrid_fixture/rgb", "--depth-topic", "/hybrid_fixture/depth",
                       "--info-topic", "/hybrid_fixture/info", "--selector-url", f"http://127.0.0.1:{server.server_port}",
                       "--cleaner-u", ".08", "--cleaner-v", ".10", "--passes", "1", "--wipe-speed", ".5",
                       "--out", str(case_out)]
            if execute:
                command.append("--execute")
            if name in ("explicit_fallback_plan", "stale_selection"):
                command.extend(["--selection-fallback", "all"])
            if name == "stale_selection":
                command.extend(["--max-decision-age", "0.000001"])
            process = subprocess.Popen(command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 45
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(scene, timeout_sec=.02)
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
                raise TimeoutError("hybrid ROS fixture timed out: " + name)
            text, _ = process.communicate(timeout=3)
            (out / (name + ".log")).write_text(text)
            if not (case_out / "summary.json").is_file():
                raise RuntimeError(text)
            summary = json.loads((case_out / "summary.json").read_text())
            assert summary["ai_requests"] == 1, text
            if name == "plan_only":
                assert process.returncode == 0 and summary["pipeline_complete"]
                assert summary["movel_commands_sent"] == 0 and not scene.goals
            elif name == "motion_complete_task_failed":
                assert process.returncode == 0 and summary["pipeline_complete"], text
                assert summary["movel_commands_sent"] >= 6 and len(scene.goals) >= 6, text
                assert summary["task_success_measured"] and summary["task_success"] is False, text
            elif name == "invalid_model_reply":
                assert process.returncode != 0 and not summary["pipeline_complete"]
                assert not scene.goals and "raw_text" in summary["error"], text
            elif name == "explicit_fallback_plan":
                assert process.returncode == 0 and summary["pipeline_complete"], text
                assert summary["rounds"][0]["decision"]["source"] == "explicit_cv_fallback", text
                assert not scene.goals, text
            elif name == "stale_selection":
                assert process.returncode != 0 and "too old" in summary["error"], text
                assert not scene.goals, text
            else:
                assert process.returncode == 3 and not summary["pipeline_complete"], text
                assert summary["rounds"][0]["executed"]["aborted"], text
                assert not any(row["kind"] in ("press", "wipe") for row in summary["move_records"]), text
            report["cases"].append({"case": name, "program_exit_code": process.returncode,
                                     "pipeline_complete": summary["pipeline_complete"],
                                     "task_success": summary["task_success"], "movel_messages_received": len(scene.goals)})
        report["complete"] = True
    finally:
        scene.destroy_node()
        rclpy.try_shutdown()
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)
        (out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        uid, gid = int(os.environ.get("WB_TEST_UID", os.getuid())), int(os.environ.get("WB_TEST_GID", os.getgid()))
        for path in [*out.rglob("*"), out, out.parent]:
            if not path.is_symlink():
                os.chown(path, uid, gid, follow_symlinks=False)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
