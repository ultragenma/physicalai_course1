"""Perceive -> plan -> execute -> verify loop for whiteboard erasing.

Run inside the ai_worker container (mock or real robot):

    python3 -m wb_eraser.run --dry-run                  # perception + plan only
    python3 -m wb_eraser.run --rounds 3                 # erase everything

Outputs go to ``runs/<timestamp>/`` (images, plan, summary JSON).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from . import perception
from .control import TOOL_DOWN, ArmClient, WorkspaceBox, execute_strokes
from .geometry import BoardFrame, transform_matrix
from .planner import PlannerConfig, merge_overlapping, plan_strokes, estimate_duration
from .robot_schema import check_timestamps


class FrameGrabber(Node):
    """Latest synchronised (colour, depth, camera_info) frame plus TF."""

    def __init__(self, image_topic: str, depth_topic: str, info_topic: str, base_frame: str = "base_link"):
        super().__init__("wb_frame_grabber")
        self.bridge = CvBridge()
        self.base_frame = base_frame
        self.latest = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.image_subscribers = [Subscriber(self, Image, image_topic, qos_profile=qos_profile_sensor_data),
                                  Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data),
                                  Subscriber(self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data)]
        self.sync = ApproximateTimeSynchronizer(self.image_subscribers, queue_size=5, slop=0.1)
        self.sync.registerCallback(self._on_frame)

    def _on_frame(self, img: Image, depth: Image, info: CameraInfo) -> None:
        self.latest = (img, depth, info)

    def grab(self, timeout: float = 5.0, fresh: bool = True):
        """Return (bgr, depth_m, K, T_base_cam, frame_id)."""
        if fresh:
            self.latest = None
        end = time.monotonic() + timeout
        while self.latest is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest is None:
            raise TimeoutError("no camera frame received")
        img, depth, info = self.latest
        stamps = [int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec) for m in (img, depth, info)]
        check_timestamps(stamps, self.get_clock().now().nanoseconds, 1.0, 0.1)
        if depth.header.frame_id != img.header.frame_id or info.header.frame_id != img.header.frame_id:
            raise ValueError("registered color/depth/CameraInfo optical frames do not match")
        bgr = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        d = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough").astype(np.float32)
        if depth.encoding in ("16UC1", "mono16"):
            d = d / 1000.0
        K = np.array(info.k, dtype=float).reshape(3, 3)
        if (d.shape != bgr.shape[:2] or not np.isfinite(K).all() or K[0, 0] <= 0 or K[1, 1] <= 0
                or depth.encoding not in ("16UC1", "mono16", "32FC1")):
            raise ValueError("invalid registered depth shape/encoding or camera calibration")
        frame = img.header.frame_id or info.header.frame_id
        stamp = rclpy.time.Time.from_msg(img.header.stamp)
        # Fill the TF buffer by spinning (lookup_transform's own timeout does not spin).
        end = time.monotonic() + timeout
        while not self.tf_buffer.can_transform(self.base_frame, frame, stamp):
            if time.monotonic() > end:
                raise TimeoutError(f"TF {self.base_frame} <- {frame} not available")
            rclpy.spin_once(self, timeout_sec=0.05)
        tf = self.tf_buffer.lookup_transform(self.base_frame, frame, stamp)
        t, q = tf.transform.translation, tf.transform.rotation
        T = transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        return bgr, d, K, T, frame


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", default="r", choices=["r", "l"])
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--stop-ratio", type=float, default=0.05, help="stop when remaining ink / initial ink < this")
    ap.add_argument("--dry-run", action="store_true", help="perceive and plan only, no arm motion")
    ap.add_argument("--no-head", action="store_true", help="do not command the head")
    ap.add_argument("--head-pitch", type=float, default=0.60)
    ap.add_argument("--head-yaw", type=float, default=-0.30)
    ap.add_argument("--ready", type=float, nargs=3, default=(0.25, -0.30, 1.05), metavar=("X", "Y", "Z"))
    ap.add_argument("--tool-offset", type=float, default=0.0, help="cleaner (felt) face distance from the EE frame (m)")
    ap.add_argument("--plane-offset", type=float, default=0.0,
                    help="shift the fitted plane along its normal toward the robot (m); negative = away")
    ap.add_argument("--board-mode", default="depth", choices=["depth", "fixed"])
    ap.add_argument("--board-x", type=float, default=0.45, help="fixed board plane x (board-mode fixed)")
    ap.add_argument("--cleaner-u", type=float, default=0.05)
    ap.add_argument("--cleaner-v", type=float, default=0.03)
    ap.add_argument("--press", type=float, default=0.004)
    ap.add_argument("--wipe-speed", type=float, default=0.12)
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--image-topic", default="/zed/zed_node/left/image_rect_color")
    ap.add_argument("--depth-topic", default="/zed/zed_node/depth/depth_registered")
    ap.add_argument("--info-topic", default="/zed/zed_node/left/camera_info")
    ap.add_argument("--out", default="")
    ap.add_argument("--workspace-x", type=float, nargs=2, default=None)
    ap.add_argument("--selector-url", default=None, help="MolmoAct2 visual target-selection service")
    ap.add_argument("--selector-timeout", type=float, default=45.0)
    ap.add_argument("--instruction", default="Erase all writing from the whiteboard.")
    ap.add_argument("--max-decision-age", type=float, default=60.0)
    ap.add_argument("--tool-quat", type=float, nargs=4, default=TOOL_DOWN, metavar=("X", "Y", "Z", "W"))
    ap.add_argument("--selection-fallback", choices=["none", "all"], default="none")
    return ap.parse_args(argv)




def main(argv=None) -> int:
    args = parse_args(argv)
    if args.rounds < 1 or not 0 < args.stop_ratio < 1 or not 0 < args.max_decision_age <= 300:
        raise ValueError("invalid round count, stopping ratio, or decision age")
    quat = np.asarray(args.tool_quat, dtype=float)
    if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-9:
        raise ValueError("tool quaternion must be finite and nonzero")
    quat = tuple(quat / np.linalg.norm(quat))
    out_dir = args.out or os.path.join("runs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    grabber = FrameGrabber(args.image_topic, args.depth_topic, args.info_topic)
    ws = WorkspaceBox.for_arm(args.arm)
    if args.workspace_x:
        ws.x = tuple(args.workspace_x)
    arm = ArmClient(args.arm, ws)
    cfg = PlannerConfig(cleaner_u=args.cleaner_u, cleaner_v=args.cleaner_v, press_depth=args.press,
                        wipe_speed=args.wipe_speed, passes=args.passes)
    summary = {"args": vars(args), "rounds": [], "pipeline_complete": False,
               "task_success": None, "task_success_measured": False, "ai_requests": 0,
               "mode": "vlm_geometry_movel" if args.selector_url else "cv_geometry_movel",
               "dry_run": args.dry_run, "contact_force_measured": False}
    code = 0
    initial_ink = None
    any_execution = False
    selector = None
    try:
        if args.selector_url:
            from .vlm_selector import TargetSelector
            selector = TargetSelector(args.selector_url, args.selector_timeout)
            summary["selector"] = selector.check_server()
        if not args.dry_run and not args.no_head:
            print(f"[head] pitch={args.head_pitch} yaw={args.head_yaw}")
            arm.set_head(args.head_pitch, args.head_yaw)
        if not args.dry_run:
            print(f"[arm] moving to ready pose {args.ready}")
            arm.wait_pose()
            if not arm.move_to(args.ready, quat=quat, speed=0.15):
                raise RuntimeError("ready pose not reached; refusing to start wiping")
        for rnd in range(args.rounds):
            print(f"===== round {rnd} =====")
            bgr, depth, K, T, frame = grabber.grab()
            per = perception.analyse(bgr, depth, K)
            if per is None:
                print("[perception] board not found; aborting")
                code = 2
                break
            if per.clipped:
                print("[perception] WARNING: board touches the image border; adjust head/base")
            if args.board_mode == "fixed":
                plane = perception.plane_from_base(T, (args.board_x, 0.0, 1.0), (-1.0, 0.0, 0.0))
            else:
                plane = per.plane
                if plane is None:
                    print("[perception] plane fit failed; aborting")
                    code = 2
                    break
                plane.point = plane.point + plane.normal * args.plane_offset
            anchor = (float(per.quad[:, 0].mean()), float(per.quad[:, 1].mean()))
            board = perception.board_frame_from_plane(plane, T, anchor, K)
            cv2.imwrite(os.path.join(out_dir, f"round{rnd}_view.png"), bgr)
            cv2.imwrite(os.path.join(out_dir, f"round{rnd}_ink.png"), perception.overlay_mask(bgr, per.ink))
            cv2.imwrite(os.path.join(out_dir, f"round{rnd}_som.png"), perception.annotate_som(bgr, per.comps))
            if initial_ink is None:
                initial_ink = max(per.ink_ratio, 1e-6)
            remaining = per.ink_ratio / initial_ink
            print(f"[perception] board {per.board_px}px wide, {len(per.comps)} components, "
                  f"ink ratio {per.ink_ratio:.4f} (remaining {remaining:.2f}), plane inliers "
                  f"{plane.inlier_ratio:.2f}, normal(base)={board.normal.round(3).tolist()}")
            rec = {"round": rnd, "components": len(per.comps), "ink_ratio": per.ink_ratio,
                   "remaining": remaining, "board": board.as_dict(), "clipped": per.clipped}
            if rnd > 0 and remaining < args.stop_ratio:
                print(f"[verify] remaining ink {remaining:.2%} < {args.stop_ratio:.0%}: done")
                summary["rounds"].append(rec)
                break
            if not per.comps:
                print("[verify] no ink components left: done")
                summary["rounds"].append(rec)
                break
            selected_components = per.comps
            if selector is not None:
                observed_at = time.monotonic()
                summary["ai_requests"] += 1
                som = perception.annotate_som(bgr, per.comps)
                try:
                    selected_ids, decision = selector.select(cv2.cvtColor(som, cv2.COLOR_BGR2RGB),
                                                              per.comps, args.instruction)
                except Exception as exc:
                    if args.selection_fallback != "all":
                        raise
                    selected_ids = [c.id for c in per.comps]
                    decision = {"source": "explicit_cv_fallback", "error": str(exc), "target_ids": selected_ids}
                if time.monotonic() - observed_at > args.max_decision_age:
                    raise ValueError("visual decision is too old for the observed board")
                rec["decision"] = decision
                with open(os.path.join(out_dir, f"round{rnd}_decision.json"), "w") as handle:
                    json.dump(decision, handle, ensure_ascii=False, indent=2)
                selected_components = [c for c in per.comps if c.id in set(selected_ids)]
            targets = perception.components_to_targets(selected_components, K, plane, T, board)
            targets = merge_overlapping(targets, gap=cfg.margin)
            strokes = plan_strokes(targets, cfg)
            n_wp = sum(len(s.waypoints) for s in strokes)
            print(f"[plan] {len(targets)} targets -> {len(strokes)} strokes, {n_wp} waypoints, "
                  f"~{estimate_duration(strokes):.0f}s")
            rec["targets"] = [asdict(t) for t in targets]
            rec["waypoints"] = n_wp
            with open(os.path.join(out_dir, f"round{rnd}_plan.json"), "w") as f:
                json.dump({"board": board.as_dict(), "targets": rec["targets"],
                           "strokes": [[asdict(w) for w in s.waypoints] for s in strokes]}, f, indent=2)
            if not targets:
                summary["rounds"].append(rec)
                break
            if not args.dry_run:
                arm.spin_for(0.1)
            rep = execute_strokes(arm, board, strokes, quat=quat, tool_offset=args.tool_offset,
                                  dry_run=args.dry_run)
            any_execution = any_execution or (not args.dry_run and rep.reached > 0)
            rec["executed"] = asdict(rep)
            print((f"[dry-run] {rep.waypoints} waypoints checked" if args.dry_run
                   else f"[exec] {rep.reached}/{rep.waypoints} waypoints completed")
                  + (f"; ABORTED: {rep.reason}" if rep.aborted else ""))
            summary["rounds"].append(rec)
            if rep.aborted:
                code = 3
                break
            if args.dry_run:
                break
            if not arm.move_to(args.ready, quat=quat, speed=0.2):
                raise RuntimeError("return to ready pose failed")
            arm.spin_for(0.5)
        # Final observation is independent of the execution counters. A completed
        # attempt with ink remaining is a task failure, not a program failure.
        if code == 0 and not args.dry_run:
            bgr, depth, K, _, _ = grabber.grab()
            final = perception.analyse(bgr, depth, K)
            if final is not None:
                remaining = final.ink_ratio / initial_ink if initial_ink is not None else None
                summary.update(task_success_measured=True,
                               task_success=bool(not final.comps or (remaining is not None and remaining < args.stop_ratio)),
                               final_ink_ratio=final.ink_ratio, final_remaining_ratio=remaining)
                cv2.imwrite(os.path.join(out_dir, "final_view.png"), bgr)
            else:
                summary["verification_error"] = "board not detected in the final observation"
        summary["pipeline_complete"] = code == 0
        summary["motion_attempted"] = any_execution
        np.save(os.path.join(out_dir, "ee_trace.npy"), np.array(arm.trace, dtype=float))
    except KeyboardInterrupt:
        print("interrupted")
        code = 130
    except Exception as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        print("[error] " + summary["error"])
        code = 4
    finally:
        summary["exit_code"] = code
        summary["movel_commands_sent"] = arm.command_counts["movel"]
        summary["head_commands_sent"] = arm.command_counts["head"]
        summary["move_records"] = [asdict(record) for record in arm.records]
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2, default=lambda value: value.tolist() if isinstance(value, np.ndarray) else str(value))
        print(f"[done] outputs in {out_dir}")
        grabber.destroy_node()
        arm.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
