"""SG2 stationary observation contract for a separate GR00T GPU server.

The model keeps the same three cameras and four joint groups as training.
This module assembles observations and validates decoded action chunks; it
has no actuator interface and does not turn joint actions into MoveL targets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .robot_schema import CAMERAS, SLICES, STATIONARY_JOINTS, check_timestamps

LANGUAGE_KEY = "annotation.human.primitive_instruction"
ACTION_HORIZON = 16


def model_modalities() -> dict:
    keys = list(SLICES)
    return {"video": {"delta_indices": [0], "modality_keys": list(CAMERAS)},
            "state": {"delta_indices": [0], "modality_keys": keys},
            "language": {"delta_indices": [0], "modality_keys": [LANGUAGE_KEY]},
            "action": {"delta_indices": list(range(ACTION_HORIZON)), "modality_keys": keys,
                       "action_configs": [{"rep": "absolute", "type": "non_eef", "format": "default"}
                                          for _ in keys]}}


def prepare_contract(snapshot: dict) -> dict:
    errors = []
    cameras = {}
    if snapshot.get("source") != "live_ros_observation_metadata":
        errors.append("expected a live ROS metadata snapshot")
    if snapshot.get("complete") is not True:
        errors.append("observation capture did not receive all requested samples")
    for name in CAMERAS:
        stream = snapshot.get("cameras", {}).get(name, {})
        sample = stream.get("sample")
        if not isinstance(sample, dict) or stream.get("count", 0) < 1 or stream.get("publisher_count", 0) < 1:
            errors.append(f"{name}: no received sample from a discovered publisher")
            continue
        if (not sample.get("frame_id") or sample.get("stamp_ns", 0) <= 0
                or sample.get("encoding") not in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8")
                or sample.get("width", 0) <= 0 or sample.get("height", 0) <= 0):
            errors.append(f"{name}: invalid frame, timestamp, image dimensions or encoding")
            continue
        cameras[name] = {key: sample[key] for key in ("frame_id", "encoding", "width", "height")}
        cameras[name]["topic"] = stream["topic"]
    stream = snapshot.get("joint_state", {})
    sample = stream.get("sample") or {}
    names = sample.get("name_order", [])
    if (stream.get("count", 0) < 1 or stream.get("publisher_count", 0) < 1
            or len(set(names)) != len(names) or sample.get("position_count") != len(names)
            or sample.get("positions_finite") is not True or sample.get("stamp_ns", 0) <= 0):
        errors.append("joint_state: missing or malformed live state sample")
    missing = [name for name in STATIONARY_JOINTS if name not in names]
    if missing:
        errors.append("joint_state: required SG2 stationary joints missing: " + ", ".join(missing))
    return {"schema_version": 1, "profile": "sg2_stationary", "observation_metadata_ready": not errors,
            "errors": errors, "cameras": cameras, "joint_state_topic": stream.get("topic"),
            "joint_order": list(STATIONARY_JOINTS),
            "live_joint_indices": [names.index(name) for name in STATIONARY_JOINTS] if not missing else None,
            "model_modalities": model_modalities(), "action_horizon": ACTION_HORIZON,
            "task_instruction": "Erase all marks from the whiteboard.",
            "server": {"host": "127.0.0.1", "port": 5555, "timeout_ms": 15000},
            "timing": {"max_sample_age_s": 0.5, "max_stamp_skew_s": 0.1, "max_result_age_s": 1.0},
            "actuation_enabled": False,
            "notes": ["Metadata availability does not validate robot identity, calibration, controller or policy performance.",
                      "No camera pixels or joint values are stored in this contract.",
                      "This client supports stationary 19D only; mobile 22D requires odometry and a separate base interface."]}


def validate_modalities(actual: dict) -> None:
    expected = model_modalities()
    for modality, wanted in expected.items():
        current = actual.get(modality)
        if not isinstance(current, dict):
            raise ValueError(f"server lacks {modality} modality")
        for key in ("modality_keys", "delta_indices"):
            if current.get(key) != wanted[key]:
                raise ValueError(f"server {modality}.{key} does not match the trained SG2 stationary contract")
    configs = actual["action"].get("action_configs")
    if not isinstance(configs, list) or len(configs) != len(SLICES):
        raise ValueError("server must expose one ActionConfig for each of the four joint groups")
    for cfg in configs:
        if not isinstance(cfg, dict) or any(str(cfg.get(k, "")).lower() != v
                                            for k, v in (("rep", "absolute"), ("type", "non_eef"), ("format", "default"))):
            raise ValueError("server ActionConfig must match ABSOLUTE / NON_EEF / DEFAULT training")


def build_observation(images: dict[str, np.ndarray], names: list[str], positions,
                      instruction: str) -> dict:
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError("task instruction must be non-empty")
    if len(set(names)) != len(names) or len(names) != len(positions):
        raise ValueError("JointState names/positions must have matching lengths and unique names")
    values = np.asarray(positions, dtype=np.float32)
    if values.ndim != 1 or not np.isfinite(values).all():
        raise ValueError("joint positions must be a finite vector")
    lookup = dict(zip(names, values))
    missing = set(STATIONARY_JOINTS) - set(lookup)
    if missing:
        raise ValueError(f"missing joints: {sorted(missing)}")
    ordered = np.array([lookup[name] for name in STATIONARY_JOINTS], dtype=np.float32)
    video = {}
    if set(images) != set(CAMERAS):
        raise ValueError("observations must contain exactly the three trained camera views")
    for name in CAMERAS:
        array = np.asarray(images[name])
        if array.dtype != np.uint8 or array.ndim != 3 or array.shape[-1] != 3 or min(array.shape[:2]) < 1:
            raise ValueError(f"{name}: expected a non-empty uint8 RGB HWC image")
        video[name] = np.ascontiguousarray(array)[None, None, ...]
    return {"video": video,
            "state": {key: ordered[start:end][None, None, ...] for key, (start, end) in SLICES.items()},
            "language": {LANGUAGE_KEY: [[instruction]]}}


def decode_action(action: dict) -> np.ndarray:
    """Return a checked (1, 16, 19) absolute-joint chunk for inspection only."""
    if not isinstance(action, dict) or set(action) != set(SLICES):
        raise ValueError("policy action keys do not match the four trained joint groups")
    parts = []
    for key, (start, end) in SLICES.items():
        array = np.asarray(action[key])
        if (array.shape != (1, ACTION_HORIZON, end - start) or array.dtype.kind not in "fi"
                or not np.isfinite(array).all()):
            raise ValueError(f"invalid action shape or values for {key}")
        parts.append(array)
    return np.concatenate(parts, axis=-1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Prepare a private three-camera SG2 client contract from live metadata")
    ap.add_argument("snapshot", type=Path)
    ap.add_argument("--out", type=Path, default=Path("runs/connection/client_contract.json"))
    args = ap.parse_args(argv)
    content = args.snapshot.read_bytes()
    contract = prepare_contract(json.loads(content))
    contract["observation_snapshot_sha256"] = hashlib.sha256(content).hexdigest()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(contract, indent=2), encoding="utf-8")
    print(f"Contract: {args.out}; observation_metadata_ready={contract['observation_metadata_ready']}")
    for error in contract["errors"]:
        print("  " + error)
    return 0 if contract["observation_metadata_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
