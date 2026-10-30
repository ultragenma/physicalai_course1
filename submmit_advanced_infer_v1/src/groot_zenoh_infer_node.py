#!/usr/bin/env python3
"""ROS 2/Zenoh observation -> NVIDIA GR00T PolicyServer -> candidate trajectory.

This bridge only publishes to a configured candidate topic. The separate action
gateway stays disarmed by default and is the only component that may forward a
candidate to a vendor-verified controller topic.
"""
from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from groot_policy_client import GrootPolicyClient
except ImportError as exc:  # helpful error on the ROS/Jazzy client environment
    raise RuntimeError(
        "The local GR00T ZMQ adapter is missing. Keep src/groot_policy_client.py "
        "beside this node and install requirements-inference-client.txt."
    ) from exc


@dataclass(frozen=True)
class ImageSample:
    stamp_ns: int
    received_monotonic: float
    frame_id: str
    rgb: np.ndarray


@dataclass(frozen=True)
class StateSample:
    stamp_ns: int
    received_monotonic: float
    positions: np.ndarray


def _required_text(mapping: dict[str, Any], key: str, label: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip() or "TODO" in value.upper():
        raise ValueError(f"{label}.{key} must be filled with a verified value")
    return value.strip()


def load_config(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if not isinstance(cfg, dict):
        raise ValueError("configuration must be a YAML mapping")

    interfaces = cfg.get("interfaces")
    policy = cfg.get("policy")
    observation = cfg.get("observation")
    timing = cfg.get("timing")
    action = cfg.get("action")
    if not all(isinstance(value, dict) for value in (interfaces, policy, observation, timing, action)):
        raise ValueError("interfaces, policy, observation, timing and action sections are required")

    for key in ("joint_state_topic", "camera_topic", "policy_action_topic"):
        _required_text(interfaces, key, "interfaces")
    command_topic = interfaces.get("command_topic", "")
    if command_topic is not None and not isinstance(command_topic, str):
        raise ValueError("interfaces.command_topic must be a string")
    if command_topic and command_topic == interfaces["policy_action_topic"]:
        raise ValueError("policy_action_topic must differ from the vendor command_topic")
    _required_text(interfaces, "camera_frame_id", "interfaces")
    for key in ("server_host", "task_instruction", "video_key", "state_key", "language_key", "action_key"):
        section = policy if key in policy else observation
        _required_text(section, key, "policy/observation")
    if policy["server_host"] != "127.0.0.1":
        raise ValueError("policy.server_host must be 127.0.0.1; use the documented IPv4 SSH local-forward")

    joint_names = cfg.get("controlled_joint_names")
    state_joint_names = observation.get("state_joint_names")
    action_joint_names = action.get("joint_names")
    for values, label in (
        (joint_names, "controlled_joint_names"),
        (state_joint_names, "observation.state_joint_names"),
        (action_joint_names, "action.joint_names"),
    ):
        if not isinstance(values, list) or not values or any(not isinstance(x, str) or not x.strip() for x in values):
            raise ValueError(f"{label} must be a non-empty ordered list")
        if len(set(values)) != len(values):
            raise ValueError(f"{label} must not contain duplicates")
        if any("TODO" in name.upper() for name in values):
            raise ValueError(f"{label} still contains placeholder joint names")
    if state_joint_names != joint_names or action_joint_names != joint_names:
        raise ValueError("state/action joint order must exactly match controlled_joint_names")

    if policy.get("action_mode") != "absolute_joint_position":
        raise ValueError(
            "this bridge only supports absolute_joint_position; NVIDIA Gr00tPolicy decodes "
            "RELATIVE actions to absolute joint targets before returning them"
        )
    for key in ("server_port", "client_timeout_ms", "chunk_index"):
        if isinstance(policy.get(key), bool) or not isinstance(policy.get(key), int):
            raise ValueError(f"policy.{key} must be an integer")
    if not 1 <= policy["server_port"] <= 65535 or policy["client_timeout_ms"] <= 0 or policy["chunk_index"] < 0:
        raise ValueError("policy server port, timeout and chunk index are outside valid ranges")

    for key in ("max_camera_age_s", "max_joint_state_age_s", "max_stamp_skew_s",
                "max_observation_age_for_action_s", "infer_period_s"):
        value = timing.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"timing.{key} must be a positive finite number")
    duration = action.get("duration_s")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration <= 0:
        raise ValueError("action.duration_s must be a positive finite number")
    if type(observation.get("history_frames")) is not int or observation["history_frames"] != 1:
        raise ValueError("this sample currently expects a checkpoint modality config with delta_indices=[0]")
    return cfg


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def validate_candidate_config(cfg: dict[str, Any]) -> None:
    """Require a known command endpoint and local joint bounds before candidate output."""
    interfaces = cfg["interfaces"]
    _required_text(interfaces, "command_topic", "interfaces")
    if interfaces["command_topic"] == interfaces["policy_action_topic"]:
        raise ValueError("policy_action_topic must differ from the vendor command_topic")

    names = cfg["controlled_joint_names"]
    limits = cfg.get("joint_limits_rad")
    velocities = cfg.get("joint_velocity_limits_rad_s")
    safety = cfg.get("safety")
    if not isinstance(limits, dict) or set(limits) != set(names):
        raise ValueError("candidate publishing requires joint_limits_rad for every controlled joint")
    if not isinstance(velocities, dict) or set(velocities) != set(names):
        raise ValueError("candidate publishing requires joint_velocity_limits_rad_s for every joint")
    if not isinstance(safety, dict):
        raise ValueError("candidate publishing requires a safety section")
    for name, bounds in limits.items():
        if not isinstance(bounds, dict):
            raise ValueError(f"joint_limits_rad.{name} must contain min and max")
        lower = _finite_number(bounds.get("min"), f"joint_limits_rad.{name}.min")
        upper = _finite_number(bounds.get("max"), f"joint_limits_rad.{name}.max")
        velocity = _finite_number(velocities[name], f"joint_velocity_limits_rad_s.{name}")
        if lower >= upper or velocity <= 0:
            raise ValueError(f"joint limits/velocity for {name} are invalid")
    max_step = _finite_number(safety.get("max_step_rad"), "safety.max_step_rad")
    minimum = _finite_number(safety.get("min_trajectory_seconds"), "safety.min_trajectory_seconds")
    maximum = _finite_number(safety.get("max_trajectory_seconds"), "safety.max_trajectory_seconds")
    duration = _finite_number(cfg["action"].get("duration_s"), "action.duration_s")
    if max_step <= 0 or minimum <= 0 or maximum < minimum:
        raise ValueError("candidate publishing safety thresholds are invalid")
    if not minimum <= duration <= maximum:
        raise ValueError("action.duration_s must be within the configured trajectory duration limits")


def stamp_to_ns(stamp) -> int:
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def image_to_rgb(msg: Image) -> np.ndarray:
    """Decode common ROS Image encodings without cv_bridge or implicit guessing."""
    enc = msg.encoding.lower()
    channel_count = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if channel_count is None:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}; use rgb8, bgr8, rgba8, bgra8 or mono8")
    min_step = int(msg.width) * channel_count
    if msg.width <= 0 or msg.height <= 0 or msg.step < min_step:
        raise ValueError("invalid image dimensions or row step")
    required_bytes = int(msg.height) * int(msg.step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < required_bytes:
        raise ValueError("image data is shorter than height * step")
    rows = raw[:required_bytes].reshape(int(msg.height), int(msg.step))
    pixels = rows[:, :min_step]
    if channel_count == 1:
        gray = pixels.reshape(int(msg.height), int(msg.width))
        rgb = np.repeat(gray[:, :, None], 3, axis=2)
    else:
        color = pixels.reshape(int(msg.height), int(msg.width), channel_count)
        if enc in {"bgr8", "bgra8"}:
            rgb = color[:, :, 2::-1]
        else:
            rgb = color[:, :, :3]
    return np.ascontiguousarray(rgb, dtype=np.uint8)


class GrootZenohInferNode(Node):
    def __init__(self, cfg: dict[str, Any], publish_candidates: bool) -> None:
        super().__init__("groot_zenoh_infer_bridge")
        self.cfg = cfg
        self.interfaces = cfg["interfaces"]
        self.policy_cfg = cfg["policy"]
        self.observation_cfg = cfg["observation"]
        self.timing = cfg["timing"]
        self.action_cfg = cfg["action"]
        self.joint_names = list(cfg["controlled_joint_names"])
        self.publish_candidates = publish_candidates
        for key in ("camera_topic", "joint_state_topic"):
            topic = self.interfaces[key]
            if not topic.startswith("/") or self.resolve_topic_name(topic) != topic:
                raise ValueError(f"{key} must be absolute and must not be ROS-remapped")
        command_topic = self.interfaces.get("command_topic", "")
        if (
            command_topic
            and self.resolve_topic_name(self.interfaces["policy_action_topic"])
            == self.resolve_topic_name(command_topic)
        ):
            raise ValueError("resolved policy_action_topic must differ from resolved command_topic")
        self._lock = threading.Lock()
        self._image: ImageSample | None = None
        self._state: StateSample | None = None
        self._last_started_image_stamp_ns = 0
        self._last_warning_at = 0.0
        self._last_inference_finished = 0.0
        self._inference_running = False
        self._subscriptions_group = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        command_topic = self.interfaces.get("command_topic", "")
        if self.publish_candidates:
            if not command_topic:
                raise ValueError("candidate publishing requires a vendor-verified JointTrajectory command_topic")
            candidate_resolved = self.resolve_topic_name(self.interfaces["policy_action_topic"])
            command_resolved = self.resolve_topic_name(command_topic)
            if not self.interfaces["policy_action_topic"].startswith("/") or candidate_resolved != self.interfaces["policy_action_topic"]:
                raise ValueError("candidate publishing requires an absolute topic with no ROS remapping")
            if not command_topic.startswith("/") or command_resolved != command_topic:
                raise ValueError("candidate publishing requires an absolute command topic with no ROS remapping")
            if candidate_resolved == command_resolved:
                raise ValueError("resolved policy_action_topic must differ from resolved command_topic")

        self.policy = GrootPolicyClient(
            host=self.policy_cfg["server_host"],
            port=self.policy_cfg["server_port"],
            timeout_ms=self.policy_cfg["client_timeout_ms"],
        )
        if not self.policy.ping():
            self.policy.close()
            raise ConnectionError("GR00T PolicyServer ping failed; check SSH tunnel and server readiness")
        try:
            self._validate_policy_modalities()
        except Exception:
            self.policy.close()
            raise
        self.get_logger().info(
            f"GR00T server reachable at {self.policy_cfg['server_host']}:{self.policy_cfg['server_port']}; "
            f"candidate publishing={'enabled' if publish_candidates else 'disabled'}"
        )

        self.create_subscription(
            Image, self.interfaces["camera_topic"], self._on_image, qos_profile_sensor_data,
            callback_group=self._subscriptions_group,
        )
        self.create_subscription(
            JointState, self.interfaces["joint_state_topic"], self._on_joint_state, qos_profile_sensor_data,
            callback_group=self._subscriptions_group,
        )
        self.candidate_publisher = None
        if self.publish_candidates:
            self.candidate_publisher = self.create_publisher(
                JointTrajectory, self.interfaces["policy_action_topic"], 10,
            )
        self.timer = self.create_timer(
            float(self.timing["infer_period_s"]), self._infer_once,
            callback_group=self._timer_group,
        )

    def _validate_policy_modalities(self) -> None:
        """Fail early if the server checkpoint schema does not match this adapter."""
        configs = self.policy.get_modality_config()
        if not isinstance(configs, dict):
            raise TypeError("GR00T get_modality_config() did not return a modality mapping")
        expected = {
            "video": self.observation_cfg["video_key"],
            "state": self.observation_cfg["state_key"],
            "language": self.observation_cfg["language_key"],
            "action": self.policy_cfg["action_key"],
        }
        for modality, key in expected.items():
            modality_config = configs.get(modality)
            modality_keys = (
                modality_config.get("modality_keys")
                if isinstance(modality_config, dict)
                else getattr(modality_config, "modality_keys", None)
            )
            if not isinstance(modality_keys, list) or modality_keys != [key]:
                raise ValueError(
                    f"this adapter supports exactly one {modality} key {key!r}; checkpoint "
                    f"modality keys {modality_keys!r}"
                )
            if modality in {"video", "state", "language"}:
                delta_indices = (
                    modality_config.get("delta_indices")
                    if isinstance(modality_config, dict)
                    else getattr(modality_config, "delta_indices", None)
                )
                if delta_indices != [0]:
                    raise ValueError(
                        f"this adapter sends one current {modality} sample; checkpoint "
                        f"{modality}.delta_indices must be [0], received {delta_indices!r}"
                    )
            elif modality == "action":
                delta_indices = (
                    modality_config.get("delta_indices")
                    if isinstance(modality_config, dict)
                    else getattr(modality_config, "delta_indices", None)
                )
                chunk_index = self.policy_cfg["chunk_index"]
                if (
                    not isinstance(delta_indices, list)
                    or not delta_indices
                    or any(type(index) is not int for index in delta_indices)
                    or chunk_index >= len(delta_indices)
                ):
                    raise ValueError(
                        f"action.delta_indices must be a non-empty integer list containing "
                        f"chunk_index={chunk_index}; received {delta_indices!r}"
                    )
                if delta_indices[chunk_index] != 0:
                    raise ValueError(
                        "this one-point adapter only supports the action sample whose "
                        "delta index is 0; choose a matching chunk_index"
                    )
                action_configs = (
                    modality_config.get("action_configs")
                    if isinstance(modality_config, dict)
                    else getattr(modality_config, "action_configs", None)
                )
                if not isinstance(action_configs, list) or len(action_configs) != 1:
                    raise ValueError("this adapter requires one explicit GR00T ActionConfig")
                action_config = action_configs[0]
                representation = (
                    action_config.get("rep")
                    if isinstance(action_config, dict)
                    else getattr(action_config, "rep", None)
                )
                representation = getattr(representation, "value", representation)
                if not isinstance(representation, str) or representation.lower() not in {"absolute", "relative"}:
                    raise ValueError(
                        "this bridge supports only GR00T ABSOLUTE/RELATIVE joint ActionConfig; "
                        f"received rep={representation!r}"
                    )
                for field, expected_value in (("type", "non_eef"), ("format", "default")):
                    value = (
                        action_config.get(field)
                        if isinstance(action_config, dict)
                        else getattr(action_config, field, None)
                    )
                    value = getattr(value, "value", value)
                    if not isinstance(value, str) or value.lower() != expected_value:
                        raise ValueError(
                            f"joint-position adapter requires ActionConfig.{field}="
                            f"{expected_value.upper()}; received {value!r}"
                        )
                if representation.lower() == "relative":
                    configured_state_key = (
                        action_config.get("state_key")
                        if isinstance(action_config, dict)
                        else getattr(action_config, "state_key", None)
                    )
                    reference_state_key = configured_state_key or key
                    if reference_state_key != self.observation_cfg["state_key"]:
                        raise ValueError(
                            "RELATIVE ActionConfig state_key must match this adapter's "
                            f"single state key {self.observation_cfg['state_key']!r}"
                        )

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_at >= 2.0:
            self.get_logger().warning(message)
            self._last_warning_at = now

    def _on_image(self, msg: Image) -> None:
        if msg.header.frame_id != self.interfaces["camera_frame_id"]:
            self._warn_throttled(
                f"Ignoring camera frame_id={msg.header.frame_id!r}; expected configured "
                f"{self.interfaces['camera_frame_id']!r}"
            )
            return
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0:
            self._warn_throttled("Ignoring camera image without a non-zero ROS header timestamp")
            return
        try:
            rgb = image_to_rgb(msg)
        except ValueError as exc:
            self._warn_throttled(str(exc))
            return
        sample = ImageSample(stamp_ns, time.monotonic(), msg.header.frame_id, rgb)
        with self._lock:
            if self._image is not None and stamp_ns <= self._image.stamp_ns:
                return
            self._image = sample

    def _on_joint_state(self, msg: JointState) -> None:
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0 or len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self._warn_throttled("Ignoring malformed JointState or missing ROS header timestamp")
            return
        positions_by_name = dict(zip(msg.name, msg.position))
        missing = [name for name in self.joint_names if name not in positions_by_name]
        if missing:
            self._warn_throttled(f"JointState lacks configured joints: {missing}")
            return
        positions = np.asarray([positions_by_name[name] for name in self.joint_names], dtype=np.float32)
        if not np.all(np.isfinite(positions)):
            self._warn_throttled("Ignoring JointState with non-finite configured joint positions")
            return
        sample = StateSample(stamp_ns, time.monotonic(), positions)
        with self._lock:
            if self._state is not None and stamp_ns <= self._state.stamp_ns:
                return
            self._state = sample

    def _snapshot(self) -> tuple[ImageSample, StateSample] | None:
        with self._lock:
            image, state = self._image, self._state
            if image is None or state is None:
                return None
            if image.stamp_ns <= self._last_started_image_stamp_ns:
                return None
            self._last_started_image_stamp_ns = image.stamp_ns
            return image, state

    def _action_vector(self, action: object) -> np.ndarray:
        key = self.policy_cfg["action_key"]
        if not isinstance(action, dict) or key not in action:
            available = list(action.keys()) if isinstance(action, dict) else type(action).__name__
            raise ValueError(f"GR00T action lacks configured key {key!r}; received {available}")
        values = np.asarray(action[key], dtype=np.float32)
        index = self.policy_cfg["chunk_index"]
        if values.ndim == 3:
            if values.shape[0] != 1 or index >= values.shape[1]:
                raise ValueError(f"action[{key!r}] has unexpected batch/horizon shape {values.shape}")
            vector = values[0, index, :]
        elif values.ndim == 2:
            if values.shape[0] != 1 or index != 0:
                raise ValueError(f"action[{key!r}] has unexpected shape for one-step output {values.shape}")
            vector = values[0, :]
        else:
            raise ValueError(f"action[{key!r}] must have shape (1,T,D) or (1,D), received {values.shape}")
        if vector.shape != (len(self.joint_names),) or not np.all(np.isfinite(vector)):
            raise ValueError(
                f"action vector must have {len(self.joint_names)} finite values in configured joint order; "
                f"received shape {vector.shape}"
            )
        return vector

    def _infer_once(self) -> None:
        now = time.monotonic()
        if self._inference_running or now - self._last_inference_finished < float(self.timing["infer_period_s"]):
            return
        samples = self._snapshot()
        if samples is None:
            self._warn_throttled("Waiting for a new synchronized camera image and JointState")
            return
        image, state = samples
        self._inference_running = True
        try:
            image_age = now - image.received_monotonic
            state_age = now - state.received_monotonic
            ros_now_ns = self.get_clock().now().nanoseconds
            image_header_age = (ros_now_ns - image.stamp_ns) / 1e9
            state_header_age = (ros_now_ns - state.stamp_ns) / 1e9
            stamp_skew = abs(image.stamp_ns - state.stamp_ns) / 1e9
            if image_age > float(self.timing["max_camera_age_s"]):
                raise ValueError(f"camera image is stale ({image_age:.3f}s)")
            if state_age > float(self.timing["max_joint_state_age_s"]):
                raise ValueError(f"JointState is stale ({state_age:.3f}s)")
            if image_header_age < -0.05 or image_header_age > float(self.timing["max_camera_age_s"]):
                raise ValueError(f"camera header timestamp is stale or from a different clock ({image_header_age:.3f}s)")
            if state_header_age < -0.05 or state_header_age > float(self.timing["max_joint_state_age_s"]):
                raise ValueError(f"JointState header timestamp is stale or from a different clock ({state_header_age:.3f}s)")
            if stamp_skew > float(self.timing["max_stamp_skew_s"]):
                raise ValueError(f"camera/JointState timestamp skew is too large ({stamp_skew:.3f}s)")

            observation = {
                "video": {
                    self.observation_cfg["video_key"]: image.rgb[None, None, ...],
                },
                "state": {
                    self.observation_cfg["state_key"]: state.positions[None, None, ...],
                },
                "language": {
                    self.observation_cfg["language_key"]: [[self.policy_cfg["task_instruction"]]],
                },
            }
            call_started = time.monotonic()
            action, _info = self.policy.get_action(observation)
            inference_s = time.monotonic() - call_started
            vector = self._action_vector(action)

            total_observation_age = time.monotonic() - min(
                image.received_monotonic, state.received_monotonic,
            )
            if total_observation_age > float(self.timing["max_observation_age_for_action_s"]):
                raise ValueError(f"discarding stale policy result ({total_observation_age:.3f}s old)")
            ros_after_inference_ns = self.get_clock().now().nanoseconds
            header_observation_age = (ros_after_inference_ns - min(image.stamp_ns, state.stamp_ns)) / 1e9
            newest_header_age = (ros_after_inference_ns - max(image.stamp_ns, state.stamp_ns)) / 1e9
            if newest_header_age < -0.05 or header_observation_age > float(self.timing["max_observation_age_for_action_s"]):
                raise ValueError(f"discarding policy result with stale/future source stamps ({header_observation_age:.3f}s)")

            if not self.publish_candidates:
                self.get_logger().info(
                    f"DRY RUN: received finite action for {len(self.joint_names)} joints "
                    f"(inference={inference_s:.3f}s); no ROS action candidate published"
                )
                return

            validate_candidate_config(self.cfg)
            duration = float(self.action_cfg["duration_s"])
            safety = self.cfg["safety"]
            for index, name in enumerate(self.joint_names):
                target = float(vector[index])
                current = float(state.positions[index])
                bounds = self.cfg["joint_limits_rad"][name]
                velocity_limit = float(self.cfg["joint_velocity_limits_rad_s"][name])
                if not float(bounds["min"]) <= target <= float(bounds["max"]):
                    raise ValueError(f"candidate {name} is outside configured joint limits")
                if abs(target - current) > float(safety["max_step_rad"]):
                    raise ValueError(f"candidate {name} exceeds configured max_step_rad")
                if abs(target - current) / duration > velocity_limit:
                    raise ValueError(f"candidate {name} exceeds configured average velocity limit")

            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in vector]
            seconds = int(duration)
            nanos = int(round((duration - seconds) * 1_000_000_000))
            if nanos >= 1_000_000_000:
                seconds += 1
                nanos -= 1_000_000_000
            point.time_from_start = Duration(sec=seconds, nanosec=nanos)
            msg.points = [point]
            if self.candidate_publisher is None:
                raise RuntimeError("candidate publisher was not enabled")
            self.candidate_publisher.publish(msg)
            self.get_logger().info(
                f"Published candidate to {self.interfaces['policy_action_topic']}; "
                f"bridge-side configured limits passed; this is not vendor safety validation "
                f"(inference={inference_s:.3f}s, mode={self.policy_cfg['action_mode']})"
            )
        except Exception as exc:  # never turn an inference error into a command
            self.get_logger().error(f"Inference/action conversion failed; no candidate published: {exc}")
        finally:
            self._inference_running = False
            self._last_inference_finished = time.monotonic()

    def close_policy(self) -> None:
        self.policy.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS 2 Zenoh to GR00T N1.7 candidate-action bridge")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--publish-candidates", action="store_true",
        help="Publish to the configured candidate topic. Without this flag, infer/log only.",
    )
    args, ros_args = parser.parse_known_args()
    cfg = load_config(args.config)
    if args.publish_candidates:
        validate_candidate_config(cfg)
    rclpy.init(args=ros_args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = GrootZenohInferNode(cfg, publish_candidates=args.publish_candidates)
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        if node is not None:
            node.close_policy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
