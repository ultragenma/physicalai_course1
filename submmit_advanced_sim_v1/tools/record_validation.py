"""Small dependency-free validators for private experiment records and exports."""
from __future__ import annotations

import math
import re
from datetime import datetime
from pathlib import Path


IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")


def require_identifier(value: object, label: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError(f"{label} must contain only letters, digits, '_' or '-' (max 64 characters)")
    return value


def require_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value


def require_recorded(value: object, label: str) -> str:
    result = require_string(value, label)
    if "todo" in result.lower() or result.strip().lower() in {"unknown", "unrecorded", "not recorded", "未記録", "未確認"}:
        raise ValueError(f"{label} still contains a placeholder or an unrecorded value")
    return result


def finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def parse_utc(value: object, label: str) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise ValueError(f"{label} must use UTC Z notation, e.g. 2026-10-24T01:23:45Z")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError(f"{label} is not a valid ISO-8601 timestamp") from exc
    if parsed.utcoffset() is None:
        raise ValueError(f"{label} must include a UTC offset")
    return parsed


def validate_metrics(metrics: object, label: str = "metrics") -> dict:
    if not isinstance(metrics, dict):
        raise ValueError(f"{label} must be a JSON object")
    if set(metrics) != {
        "available", "before_target_ink_pixels", "residual_target_ink_pixels",
        "target_roi_erasure_ratio", "outside_ink_mask_change_ratio",
    }:
        raise ValueError(f"{label} has missing or unexpected fields")
    if not isinstance(metrics["available"], bool):
        raise ValueError(f"{label}.available must be boolean")
    numeric_fields = (
        "before_target_ink_pixels", "residual_target_ink_pixels",
        "target_roi_erasure_ratio", "outside_ink_mask_change_ratio",
    )
    if not metrics["available"]:
        if any(metrics[key] is not None for key in numeric_fields):
            raise ValueError(f"{label} values must be null when available is false")
        return metrics

    before = metrics["before_target_ink_pixels"]
    residual = metrics["residual_target_ink_pixels"]
    if isinstance(before, bool) or not isinstance(before, int) or before <= 0:
        raise ValueError(f"{label}.before_target_ink_pixels must be a positive integer")
    if isinstance(residual, bool) or not isinstance(residual, int) or residual < 0:
        raise ValueError(f"{label}.residual_target_ink_pixels must be a non-negative integer")
    erasure = finite_number(metrics["target_roi_erasure_ratio"], f"{label}.target_roi_erasure_ratio")
    outside = finite_number(metrics["outside_ink_mask_change_ratio"], f"{label}.outside_ink_mask_change_ratio")
    if erasure > 1.0 or outside < 0.0 or outside > 1.0:
        raise ValueError(f"{label} ratios are outside their permitted ranges")
    if not math.isclose(erasure, 1.0 - residual / before, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError(f"{label}.target_roi_erasure_ratio does not match pixel counts")
    return metrics


def validate_metadata(metadata: object, require_public_review: bool = False) -> dict:
    if not isinstance(metadata, dict) or metadata.get("schema_version") != "1.0":
        raise ValueError("metadata must be a schema_version 1.0 JSON object")
    require_identifier(metadata.get("run_id"), "metadata.run_id")
    parse_utc(metadata.get("recorded_at_utc"), "metadata.recorded_at_utc")
    execution = metadata.get("execution")
    software = metadata.get("software")
    robot = metadata.get("robot")
    task = metadata.get("task")
    privacy = metadata.get("privacy_review")
    if not all(isinstance(value, dict) for value in (execution, software, robot, task, privacy)):
        raise ValueError("metadata.execution/software/robot/task/privacy_review must be objects")
    if execution.get("kind") not in {"simulation", "real_robot"}:
        raise ValueError("metadata.execution.kind must be simulation or real_robot")
    for key in ("simulator", "asset_source", "asset_revision", "asset_variant", "scene_revision"):
        require_recorded(execution.get(key), f"metadata.execution.{key}")
    if execution["kind"] == "simulation" and execution["simulator"] not in {"gazebo", "mujoco", "isaac_lab", "other"}:
        raise ValueError("metadata.execution.simulator is not a supported value")
    for key in ("ros_distro", "robot_software_version", "policy_runtime", "policy_runtime_revision", "policy_model", "checkpoint_id"):
        require_recorded(software.get(key), f"metadata.software.{key}")
    gpu = software.get("gpu")
    if not isinstance(gpu, dict):
        raise ValueError("metadata.software.gpu must be an object")
    require_recorded(gpu.get("name"), "metadata.software.gpu.name")
    require_recorded(gpu.get("driver_version"), "metadata.software.gpu.driver_version")
    vram = gpu.get("vram_gb")
    if vram is not None and finite_number(vram, "metadata.software.gpu.vram_gb") < 0:
        raise ValueError("metadata.software.gpu.vram_gb must be non-negative")
    for key in ("model", "asset_license", "interface_contract_revision"):
        require_recorded(robot.get(key), f"metadata.robot.{key}")
    for key in ("instruction", "board_setup_id", "mask_generation_method", "calibration_revision"):
        require_recorded(task.get(key), f"metadata.task.{key}")
    privacy_keys = (
        "contains_personal_information", "contains_internal_network_information",
        "contains_confidential_information",
    )
    for key in privacy_keys:
        if privacy.get(key) is not True and privacy.get(key) is not False and privacy.get(key) is not None:
            raise ValueError(f"metadata.privacy_review.{key} must be boolean or null")
    require_recorded(privacy.get("public_safe_summary"), "metadata.privacy_review.public_safe_summary")
    if require_public_review and any(privacy.get(key) is not False for key in privacy_keys):
        raise ValueError("public report generation requires all privacy flags to be reviewed and false")
    return metadata


def validate_trial(trial: object, expected_run_id: str) -> dict:
    if not isinstance(trial, dict) or trial.get("schema_version") != "1.0":
        raise ValueError("trial must be a schema_version 1.0 JSON object")
    run_id = require_identifier(trial.get("run_id"), "trial.run_id")
    trial_id = require_identifier(trial.get("trial_id"), "trial.trial_id")
    if run_id != expected_run_id:
        raise ValueError(f"trial {trial_id} belongs to run {run_id}, expected {expected_run_id}")
    if trial.get("status") not in {"completed", "failed", "aborted", "invalid"}:
        raise ValueError(f"trial {trial_id} has an invalid status")
    started = parse_utc(trial.get("started_at_utc"), f"trial {trial_id} started_at_utc")
    ended = parse_utc(trial.get("ended_at_utc"), f"trial {trial_id} ended_at_utc")
    duration = finite_number(trial.get("duration_seconds"), f"trial {trial_id} duration_seconds")
    measured_duration = (ended - started).total_seconds()
    if duration < 0 or ended < started or not math.isclose(duration, measured_duration, abs_tol=1e-6):
        raise ValueError(f"trial {trial_id} duration or timestamps are inconsistent")
    validate_metrics(trial.get("metrics"), f"trial {trial_id}.metrics")
    evidence = trial.get("evidence")
    required_evidence = {
        "before_mask_sha256", "after_mask_sha256", "board_mask_sha256",
        "target_roi_mask_sha256", "video_id_or_redacted_path",
    }
    if not isinstance(evidence, dict) or set(evidence) != required_evidence:
        raise ValueError(f"trial {trial_id}.evidence has missing or unexpected fields")
    for key, value in evidence.items():
        if key.endswith("sha256") and value is not None and (not isinstance(value, str) or not SHA256.fullmatch(value)):
            raise ValueError(f"trial {trial_id}.evidence.{key} must be a lowercase SHA-256 or null")
        if key == "video_id_or_redacted_path" and value is not None and not isinstance(value, str):
            raise ValueError(f"trial {trial_id}.evidence.{key} must be a string or null")
    if not isinstance(trial.get("operator_notes"), str):
        raise ValueError(f"trial {trial_id}.operator_notes must be a string")
    return trial


def ensure_artifacts_run_path(path: Path, run_id: str, label: str = "path") -> Path:
    require_identifier(run_id, "run_id")
    run_root = (Path("artifacts") / run_id).resolve()
    target = path.resolve()
    try:
        target.relative_to(run_root)
    except ValueError as exc:
        raise ValueError(f"{label} must stay under artifacts/{run_id}/") from exc
    return target


def ensure_artifacts_output(path: Path, run_id: str) -> Path:
    return ensure_artifacts_run_path(path, run_id, "output file")
