"""Cyclo Control MoveL client plus head / lift commands (ROS 2 Jazzy, rclpy).

Cyclo Control's ``ai_worker_movel_controller`` subscribes to
``/{r,l}_goal_move`` (robotis_interfaces/MoveL) and publishes the current
end-effector pose on ``/{r,l}_gripper_pose``. Goals are interpolated linearly
over ``time_from_start`` from the pose at the time of arrival.

Safety: every goal is checked against a :class:`WorkspaceBox` before it is
published. Goals outside the box raise :class:`WorkspaceError`.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robotis_interfaces.msg import MoveL
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .motion import TOOL_DOWN, ExecutionReport, WorkspaceBox, WorkspaceError, execute_strokes


def _duration(sec: float) -> Duration:
    sec = max(sec, 0.0)
    whole = int(sec)
    return Duration(sec=whole, nanosec=int((sec - whole) * 1e9))


@dataclass
class MoveRecord:
    t: float
    target: np.ndarray
    kind: str
    reached: bool
    error: float


class ArmClient(Node):
    """MoveL client for one arm plus head and lift helpers."""

    def __init__(self, arm: str = "r", workspace: WorkspaceBox | None = None,
                 node_name: str | None = None):
        super().__init__(node_name or f"wb_arm_client_{arm}")
        if arm not in ("r", "l"):
            raise ValueError("arm must be 'r' or 'l'")
        self.arm = arm
        self.workspace = workspace or WorkspaceBox.for_arm(arm)
        self._pose: PoseStamped | None = None
        self._pose_time = 0.0
        self.trace: list[tuple[float, float, float, float]] = []
        self.records: list[MoveRecord] = []
        self.command_counts = {"movel": 0, "head": 0, "lift": 0}
        self._goal_pub = self.create_publisher(MoveL, f"/{arm}_goal_move", 10)
        self._pose_sub = self.create_subscription(PoseStamped, f"/{arm}_gripper_pose", self._on_pose, 10)
        self._head_pub = self.create_publisher(
            JointTrajectory, "/leader/joystick_controller_left/joint_trajectory", 10)
        self._lift_pub = self.create_publisher(
            JointTrajectory, "/leader/joystick_controller_right/joint_trajectory", 10)

    # ------------------------------------------------------------------ state
    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg
        self._pose_time = time.monotonic()
        p = msg.pose.position
        self.trace.append((time.monotonic(), p.x, p.y, p.z))

    def spin_for(self, sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(end - time.monotonic(), 0.0)))

    def wait_pose(self, timeout: float = 5.0) -> np.ndarray:
        end = time.monotonic() + timeout
        while self._pose is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._pose is None:
            raise TimeoutError(f"no pose on /{self.arm}_gripper_pose; is Cyclo MoveL running?")
        return self.position

    @property
    def position(self) -> np.ndarray:
        if self._pose is None:
            raise RuntimeError("pose not received yet")
        p = self._pose.pose.position
        return np.array([p.x, p.y, p.z])

    @property
    def orientation(self) -> np.ndarray:
        if self._pose is None:
            raise RuntimeError("pose not received yet")
        q = self._pose.pose.orientation
        return np.array([q.x, q.y, q.z, q.w])

    # ------------------------------------------------------------------ motion
    def move_to(self, xyz, quat=TOOL_DOWN, speed: float = 0.2, min_time: float = 0.5,
                tol: float = 0.006, settle: float = 0.15, wait: str = "pose",
                timeout_extra: float = 3.0, kind: str = "move") -> bool:
        """Publish one MoveL goal and wait for it.

        wait='pose': return when the measured pose is within ``tol`` (or timeout).
        wait='time': return after the interpolation time plus ``settle`` (use
        when the goal is intentionally inside a surface and cannot be reached).
        """
        target = np.asarray(xyz, dtype=float)
        if not self.workspace.contains(target):
            raise WorkspaceError(f"goal {target.round(3).tolist()} outside workspace {self.workspace}")
        start = self.wait_pose()
        dist = float(np.linalg.norm(target - start))
        duration = max(min_time, dist / max(speed, 1e-3))

        msg = MoveL()
        msg.pose.header.frame_id = "base_link"
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, target)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = map(float, quat)
        msg.time_from_start = _duration(duration)
        self._goal_pub.publish(msg)
        self.command_counts["movel"] += 1

        t0 = time.monotonic()
        reached = False
        if wait == "time":
            self.spin_for(duration + settle)
            reached = True
        else:
            deadline = t0 + duration + timeout_extra
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self._pose is not None and np.linalg.norm(self.position - target) < tol:
                    reached = True
                    break
            if reached and settle > 0:
                self.spin_for(settle)
        err = float(np.linalg.norm(self.position - target))
        self.records.append(MoveRecord(time.monotonic(), target, kind, reached, err))
        if not reached:
            self.get_logger().warn(f"goal {target.round(3).tolist()} not reached (err {err*1000:.1f} mm)")
        return reached

    def move_relative(self, dxyz, **kw) -> bool:
        return self.move_to(self.wait_pose() + np.asarray(dxyz, float), **kw)

    # ------------------------------------------------------------------ head / lift
    def set_head(self, pitch: float, yaw: float = 0.0, duration: float = 2.0) -> None:
        """head_joint1 = pitch (positive looks down, max ~0.69 rad), head_joint2 = yaw."""
        pitch = float(np.clip(pitch, -0.23, 0.69))
        yaw = float(np.clip(yaw, -0.35, 0.35))
        traj = JointTrajectory()
        traj.joint_names = ["head_joint1", "head_joint2"]
        pt = JointTrajectoryPoint()
        pt.positions = [pitch, yaw]
        pt.time_from_start = _duration(duration)
        traj.points.append(pt)
        self._head_pub.publish(traj)
        self.command_counts["head"] += 1
        self.spin_for(duration + 0.2)

    def set_lift(self, z: float, duration: float = 3.0) -> None:
        """lift_joint range is [-0.5, 0.0] m (negative = lower the torso)."""
        z = float(np.clip(z, -0.5, 0.0))
        traj = JointTrajectory()
        traj.joint_names = ["lift_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [z]
        pt.time_from_start = _duration(duration)
        traj.points.append(pt)
        self._lift_pub.publish(traj)
        self.command_counts["lift"] += 1
        self.spin_for(duration + 0.2)


def quat_rotate_y(angle_rad: float) -> tuple[float, float, float, float]:
    """Quaternion for a rotation about base y (tool pitch)."""
    return (0.0, math.sin(angle_rad / 2.0), 0.0, math.cos(angle_rad / 2.0))
