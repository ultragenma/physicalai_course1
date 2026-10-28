"""MolmoAct2 observation/action contracts. No ROS, Torch or actuator imports."""

from __future__ import annotations

import base64
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from .robot_schema import ARM_RIGHT, CAMERAS, STATIONARY_JOINTS

SCHEMA_VERSION = 1
MAX_BODY = 32 * 1024 * 1024
MAX_PIXELS = 4096 * 4096


@dataclass(frozen=True)
class Profile:
    name: str
    kind: str
    norm_tag: str
    camera_order: tuple[str, ...]
    joint_order: tuple[str, ...]
    action_names: tuple[str, ...]

    @classmethod
    def load(cls, path: Path) -> "Profile":
        value = json.loads(path.read_text())
        profile = cls(value["name"], value["kind"], value["norm_tag"], tuple(value["camera_order"]),
                      tuple(value["joint_order"]), tuple(value["action_names"]))
        if not profile.name or not profile.norm_tag or len(set(profile.joint_order)) != len(profile.joint_order):
            raise ValueError("invalid profile identity or duplicated joint names")
        if not 1 <= len(profile.camera_order) <= 3 or not set(profile.camera_order) <= set(CAMERAS):
            raise ValueError("profile must name one to three known camera slots")
        if profile.kind == "droid_probe":
            if (profile.norm_tag != "franka_droid" or profile.joint_order != ARM_RIGHT
                    or profile.action_names != tuple(f"franka_joint{i}" for i in range(1, 8)) + ("franka_gripper",)
                    or profile.camera_order != (CAMERAS[0], CAMERAS[0], CAMERAS[2])):
                raise ValueError("DROID probe requires the explicit right-arm diagnostic contract")
        elif profile.kind == "sg2_native":
            if profile.joint_order != STATIONARY_JOINTS or profile.action_names != STATIONARY_JOINTS:
                raise ValueError("SG2 native policy must preserve the named 19D stationary joint order")
            if profile.camera_order != CAMERAS:
                raise ValueError("SG2 native policy must preserve its three training camera views")
        else:
            raise ValueError("profile kind must be droid_probe or sg2_native")
        return profile


def _dimension(stats):
    if not isinstance(stats, dict):
        raise ValueError("checkpoint must contain normalization statistics")
    lengths = []
    for key in ("mean", "std", "min", "max", "q01", "q99", "q10", "q90"):
        if key in stats:
            values = np.asarray(stats[key], dtype=np.float64)
            if values.ndim != 1 or not len(values) or not np.isfinite(values).all():
                raise ValueError("normalization statistics must be finite vectors")
            lengths.append(len(values))
    if not lengths or len(set(lengths)) != 1:
        raise ValueError("normalization statistics have missing or inconsistent dimensions")
    return lengths[0]


def checkpoint_schema(checkpoint: Path, profile: Profile) -> dict:
    """Read the actual tag/horizon, never reuse GR00T's fixed 16-step format."""
    config = json.loads((checkpoint / "config.json").read_text())
    if config.get("model_type") != "molmoact2" or not config.get("add_action_expert"):
        raise ValueError("a continuous-action MolmoAct2 checkpoint is required")
    raw = (checkpoint / config.get("norm_stats_filename", "norm_stats.json")).read_bytes()
    stats = json.loads(raw)
    metadata = stats.get("metadata_by_tag", {}).get(profile.norm_tag)
    if not isinstance(metadata, dict):
        raise ValueError(f"checkpoint has no normalization tag {profile.norm_tag!r}; AIWorker needs its own fine-tuned checkpoint")
    state_dim, action_dim = _dimension(metadata.get("state_stats")), _dimension(metadata.get("action_stats"))
    if state_dim != len(profile.joint_order) or action_dim != len(profile.action_names):
        raise ValueError("checkpoint normalization dimensions do not match the selected embodiment")
    maximum = int(config["max_action_horizon"])
    horizon = int(metadata.get("action_horizon") or maximum)
    steps = int(metadata.get("n_action_steps") or horizon)
    if not 1 <= steps <= horizon <= maximum or action_dim > int(config["max_action_dim"]):
        raise ValueError("invalid checkpoint action horizon or dimension")
    if profile.kind == "sg2_native":
        # Dimensions alone cannot establish which joints a learned policy controls.
        if (metadata.get("robot_type") != "robotis_aiworker_sg2"
                or metadata.get("state_joint_names") != list(profile.joint_order)
                or metadata.get("action_joint_names") != list(profile.action_names)
                or metadata.get("camera_order") != list(profile.camera_order)
                or metadata.get("action_representation") != "absolute_joint_positions"):
            raise ValueError("native checkpoint must declare AIWorker identity, named state/action order, cameras and representation")
    return {"schema_version": SCHEMA_VERSION, "model_type": "molmoact2", "profile": profile.name,
            "kind": profile.kind, "norm_tag": profile.norm_tag, "state_dim": state_dim, "action_dim": action_dim,
            "action_horizon": horizon, "n_action_steps": steps, "camera_order": list(profile.camera_order),
            "joint_order": list(profile.joint_order), "action_names": list(profile.action_names),
            "action_representation": "absolute_joint_positions" if profile.kind == "sg2_native" else "franka_droid_absolute_joints_and_gripper",
            "aiworker_action_layout": profile.kind == "sg2_native", "actuation_enabled": False,
            "norm_stats_sha256": hashlib.sha256(raw).hexdigest(),
            "source_robot": "robotis_aiworker_sg2", "policy_robot": "franka_droid" if profile.kind == "droid_probe" else "robotis_aiworker_sg2",
            "state_semantics_verified": profile.kind != "droid_probe"}


def ordered_state(names, positions, wanted):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("JointState names/positions must have matching lengths and unique names")
    values = np.asarray(positions, dtype=np.float32)
    if values.shape != (len(names),) or not np.isfinite(values).all():
        raise ValueError("joint positions must be a finite vector")
    lookup = dict(zip(names, values))
    if not set(wanted) <= set(lookup):
        raise ValueError("required AIWorker joints are missing")
    return np.array([lookup[name] for name in wanted], dtype=np.float32)


def encode_image(image):
    array = np.asarray(image)
    if (array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3
            or min(array.shape[:2]) < 1 or array.shape[0] * array.shape[1] > MAX_PIXELS):
        raise ValueError("images must be bounded nonempty uint8 RGB HWC arrays")
    stream = io.BytesIO()
    Image.fromarray(array).save(stream, format="PNG", compress_level=1)
    return base64.b64encode(stream.getvalue()).decode("ascii")


def decode_image(value):
    if not isinstance(value, str) or len(value) > MAX_BODY:
        raise ValueError("invalid image payload")
    raw = base64.b64decode(value, validate=True)
    with Image.open(io.BytesIO(raw)) as image:
        if image.format != "PNG" or image.mode != "RGB" or image.width * image.height > MAX_PIXELS:
            raise ValueError("only bounded RGB PNG images are accepted")
        return np.array(image, dtype=np.uint8)


def make_request(images, names, positions, task, schema, request_id, stamps_ns=None):
    if not isinstance(task, str) or not task.strip() or len(task) > 2000:
        raise ValueError("a nonempty instruction of at most 2000 characters is required")
    required = set(schema["camera_order"])
    if not required <= set(images):
        raise ValueError("required cameras are missing")
    state = ordered_state(names, positions, schema["joint_order"])
    return {"schema_version": SCHEMA_VERSION, "profile": schema["profile"], "request_id": request_id,
            "task": task, "joint_order": schema["joint_order"], "state": state.tolist(),
            "images": {name: encode_image(images[name]) for name in sorted(required)},
            "observation_stamps_ns": list(stamps_ns or [])}


def parse_request(body, schema):
    if not isinstance(body, dict) or body.get("schema_version") != SCHEMA_VERSION or body.get("profile") != schema["profile"]:
        raise ValueError("request protocol/profile mismatch")
    if (not isinstance(body.get("request_id"), str) or not 1 <= len(body["request_id"]) <= 128
            or not isinstance(body.get("task"), str) or not body["task"].strip() or len(body["task"]) > 2000):
        raise ValueError("invalid request ID or instruction")
    if body.get("joint_order") != schema["joint_order"]:
        raise ValueError("request joint order differs from the checkpoint profile")
    state = np.asarray(body["state"], dtype=np.float32)
    if state.shape != (schema["state_dim"],) or not np.isfinite(state).all():
        raise ValueError("invalid state shape or non-finite state")
    images = body.get("images")
    if not isinstance(images, dict) or set(images) != set(schema["camera_order"]):
        raise ValueError("request camera set differs from the checkpoint profile")
    decoded = {name: decode_image(value) for name, value in images.items()}
    return [decoded[name] for name in schema["camera_order"]], state, body["task"]


def check_actions(actions, schema):
    values = np.asarray(actions, dtype=np.float32)
    if values.shape != (schema["n_action_steps"], schema["action_dim"]) or not np.isfinite(values).all():
        raise ValueError("returned action chunk has wrong shape or non-finite values")
    return values


def validate_schema(actual, profile):
    if not isinstance(actual, dict) or actual.get("schema_version") != SCHEMA_VERSION or actual.get("model_type") != "molmoact2":
        raise ValueError("server is not the expected MolmoAct2 service")
    for key, expected in [("profile", profile.name), ("kind", profile.kind), ("norm_tag", profile.norm_tag),
                          ("joint_order", list(profile.joint_order)), ("action_names", list(profile.action_names)),
                          ("camera_order", list(profile.camera_order)), ("state_dim", len(profile.joint_order)),
                          ("action_dim", len(profile.action_names)), ("actuation_enabled", False),
                          ("aiworker_action_layout", profile.kind == "sg2_native")]:
        if actual.get(key) != expected:
            raise ValueError(f"server {key} does not match the selected profile")
    if not 1 <= actual.get("n_action_steps", 0) <= actual.get("action_horizon", 0) <= 100:
        raise ValueError("server returned an invalid action horizon")
    return actual
