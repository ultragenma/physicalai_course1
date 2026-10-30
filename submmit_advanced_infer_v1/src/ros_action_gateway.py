#!/usr/bin/env python3
"""ROS 2 candidate-trajectory filter for a separately validated policy.

This node is not a robot safety controller. It checks a small subset of
trajectory properties before forwarding one candidate point. It does not
implement emergency stop, workspace/collision/contact-force/acceleration
limits, network authentication, or controller-health monitoring.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


def load_config(path: str) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError("configuration must be a mapping")
    return cfg


def is_canonical_nonnegative_time(value) -> bool:
    return (
        type(value.sec) is int
        and type(value.nanosec) is int
        and value.sec >= 0
        and 0 <= value.nanosec < 1_000_000_000
    )


class ActionGateway(Node):
    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__("aiworker_action_gateway")
        self.cfg = cfg
        interface = cfg["interfaces"]
        safety = cfg["safety"]
        self.names = list(cfg["controlled_joint_names"])
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError("controlled_joint_names must be non-empty and unique")
        if not interface["joint_state_topic"]:
            raise ValueError("joint_state_topic must be configured")
        candidate_topic = interface.get("policy_action_topic", "")
        command_topic = interface.get("command_topic", "")
        if not candidate_topic:
            raise ValueError("policy_action_topic must be configured")
        for label, topic in (
            ("joint_state_topic", interface["joint_state_topic"]),
            ("policy_action_topic", candidate_topic),
        ):
            if not topic.startswith("/") or self.resolve_topic_name(topic) != topic:
                raise ValueError(f"{label} must be absolute and must not be ROS-remapped")
        if command_topic and candidate_topic == command_topic:
            raise ValueError("policy_action_topic must differ from the vendor command_topic")
        if (
            command_topic
            and self.resolve_topic_name(candidate_topic) == self.resolve_topic_name(command_topic)
        ):
            raise ValueError("resolved policy_action_topic must differ from resolved command_topic")
        if not safety["require_operator_arm"]:
            raise ValueError("require_operator_arm must remain true")
        if safety.get("arm_parameter") is not False:
            raise ValueError("safety.arm_parameter must remain false; use an explicit ROS parameter override to arm")
        arm_value = self.declare_parameter("arm_parameter", False).value
        if type(arm_value) is not bool:
            raise ValueError("arm_parameter must be an explicit boolean")
        self.arm = arm_value
        if self.arm and not command_topic:
            raise ValueError("command_topic must be vendor-verified before arming the gateway")
        if self.arm and (
            not command_topic.startswith("/")
            or self.resolve_topic_name(command_topic) != command_topic
        ):
            raise ValueError("armed command_topic must be absolute and must not be ROS-remapped")
        if self.arm:
            self.get_logger().warning("ARMED: only validated single-point trajectories will be forwarded")
        else:
            self.get_logger().warning("DISARMED: all policy candidates will be rejected")
        self.safety = safety
        self.limits = cfg["joint_limits_rad"]
        self.velocity_limits = cfg["joint_velocity_limits_rad_s"]
        if set(self.names) != set(self.limits):
            raise ValueError("joint_limits_rad must specify exactly the controlled joint names")
        if set(self.names) != set(self.velocity_limits):
            raise ValueError("joint_velocity_limits_rad_s must specify exactly the controlled joint names")
        self.max_step = self._positive_finite("max_step_rad")
        self.min_trajectory_seconds = self._positive_finite("min_trajectory_seconds")
        self.max_trajectory_seconds = self._positive_finite("max_trajectory_seconds")
        if self.min_trajectory_seconds > self.max_trajectory_seconds:
            raise ValueError("min_trajectory_seconds cannot exceed max_trajectory_seconds")
        self.joint_state_timeout_ns = int(self._positive_finite("joint_state_timeout_seconds") * 1e9)
        self.policy_action_timeout_ns = int(self._positive_finite("policy_action_timeout_seconds") * 1e9)
        for name, bounds in self.limits.items():
            lower, upper = float(bounds["min"]), float(bounds["max"])
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError(f"invalid finite joint bounds for {name}")
            velocity_limit = float(self.velocity_limits[name])
            if not math.isfinite(velocity_limit) or velocity_limit <= 0.0:
                raise ValueError(f"invalid positive finite velocity limit for {name}")
        self.current: dict[str, float] = {}
        self.last_joint_state_received_monotonic_ns: int | None = None
        self.last_joint_state_stamp_ns: int | None = None
        self.create_subscription(JointState, interface["joint_state_topic"], self.on_joint_state, qos_profile_sensor_data)
        self.create_subscription(JointTrajectory, interface["policy_action_topic"], self.on_candidate, 10)
        # A disarmed gateway does not even register as a publisher on the
        # vendor command topic. This prevents accidental controller discovery.
        self.publisher = (
            self.create_publisher(JointTrajectory, command_topic, 10)
            if self.arm else None
        )

    def _positive_finite(self, key: str) -> float:
        value = float(self.safety[key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"safety.{key} must be positive and finite")
        return value

    def on_joint_state(self, msg: JointState) -> None:
        now_ns = self.get_clock().now().nanoseconds
        state_stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if is_canonical_nonnegative_time(msg.header.stamp) else 0
        )
        state_age_ns = now_ns - state_stamp_ns
        if (
            len(msg.name) != len(msg.position)
            or len(set(msg.name)) != len(msg.name)
            or any(not name or not math.isfinite(position) for name, position in zip(msg.name, msg.position))
            or state_stamp_ns <= 0
            or state_age_ns > self.joint_state_timeout_ns
            or state_age_ns < -50_000_000
        ):
            self.current = {}
            self.last_joint_state_received_monotonic_ns = None
            self.last_joint_state_stamp_ns = None
            self.get_logger().warning("Ignoring malformed or stale JointState; gateway remains fail-closed")
            return
        self.current = dict(zip(msg.name, msg.position))
        self.last_joint_state_received_monotonic_ns = time.monotonic_ns()
        self.last_joint_state_stamp_ns = state_stamp_ns

    def reject(self, reason: str) -> None:
        self.get_logger().error(f"Rejected policy trajectory: {reason}")

    def on_candidate(self, msg: JointTrajectory) -> None:
        if not self.arm:
            return self.reject("gateway is disarmed")
        now_ns = self.get_clock().now().nanoseconds
        if self.last_joint_state_received_monotonic_ns is None or self.last_joint_state_stamp_ns is None:
            return self.reject("joint state missing or stale")
        receipt_age_ns = time.monotonic_ns() - self.last_joint_state_received_monotonic_ns
        state_age_ns = now_ns - self.last_joint_state_stamp_ns
        if (
            receipt_age_ns < 0
            or receipt_age_ns > self.joint_state_timeout_ns
            or state_age_ns > self.joint_state_timeout_ns
            or state_age_ns < -50_000_000
        ):
            return self.reject("joint state header or receipt time is stale or from a different clock")
        if not is_canonical_nonnegative_time(msg.header.stamp):
            return self.reject("policy action timestamp has non-canonical sec/nanosec fields")
        action_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        action_age_ns = now_ns - action_stamp_ns
        if action_stamp_ns <= 0 or action_age_ns > self.policy_action_timeout_ns or action_age_ns < -50_000_000:
            return self.reject("policy action timestamp missing, stale, or too far in the future")
        if list(msg.joint_names) != self.names:
            return self.reject("candidate joint order differs from configured order")
        if len(msg.points) != 1:
            return self.reject("only one candidate point is allowed")
        point = msg.points[0]
        if len(point.positions) != len(self.names) or any(not math.isfinite(x) for x in point.positions):
            return self.reject("invalid position vector")
        if point.velocities or point.accelerations or point.effort:
            return self.reject("position-only trajectory points are required")
        for name, target in zip(self.names, point.positions, strict=True):
            if name not in self.current:
                return self.reject(f"no current state for {name}")
            bound = self.limits[name]
            if target < float(bound["min"]) or target > float(bound["max"]):
                return self.reject(f"{name} outside configured limits")
            if abs(target - self.current[name]) > self.max_step:
                return self.reject(f"{name} delta exceeds max_step_rad")
        if not is_canonical_nonnegative_time(point.time_from_start):
            return self.reject("trajectory duration has non-canonical sec/nanosec fields")
        duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        if duration < self.min_trajectory_seconds or duration > self.max_trajectory_seconds:
            return self.reject("trajectory duration is outside the configured range")
        for name, target in zip(self.names, point.positions, strict=True):
            average_velocity = abs(target - self.current[name]) / duration
            if average_velocity > float(self.velocity_limits[name]):
                return self.reject(f"{name} average velocity exceeds its configured limit")
        if self.publisher is None:
            return self.reject("no command publisher exists while gateway is disarmed")
        outbound = JointTrajectory()
        outbound.header = msg.header
        outbound.joint_names = self.names
        outbound.points = [point]
        self.publisher.publish(outbound)
        self.get_logger().info("Forwarded one bounded trajectory point")


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    try:
        node = ActionGateway(load_config(args.config))
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
