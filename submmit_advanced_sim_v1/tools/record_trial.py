#!/usr/bin/env python3
"""Combine one metrics JSON and operator-recorded facts into one trial JSON.

This script has no ROS or simulator dependency. It never reads images/videos,
only the metrics JSON emitted by erase_assessment.py. Keep output under the
ignored artifacts/ directory until it has passed the privacy review.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from record_validation import ensure_artifacts_output, parse_utc, validate_metadata, validate_metrics, validate_trial


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description="Create one machine-readable whiteboard-erasure trial record")
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, help="JSON from erase_assessment.py; omit only when metrics are unavailable")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--status", choices=("completed", "failed", "aborted", "invalid"), required=True)
    parser.add_argument("--started-at-utc", required=True)
    parser.add_argument("--ended-at-utc", required=True)
    parser.add_argument("--video-id-or-redacted-path", default=None)
    parser.add_argument("--before-mask-sha256", default=None)
    parser.add_argument("--after-mask-sha256", default=None)
    parser.add_argument("--board-mask-sha256", default=None)
    parser.add_argument("--target-roi-mask-sha256", default=None)
    parser.add_argument("--operator-notes", default="")
    args = parser.parse_args()

    metadata = validate_metadata(load_json(args.metadata))
    run_id = metadata["run_id"]
    started, ended = parse_utc(args.started_at_utc, "--started-at-utc"), parse_utc(args.ended_at_utc, "--ended-at-utc")
    if ended < started:
        raise ValueError("ended time is before started time")
    if args.metrics:
        raw = load_json(args.metrics)
        metrics = {
            "available": True,
            "before_target_ink_pixels": raw.get("before_target_ink_pixels"),
            "residual_target_ink_pixels": raw.get("residual_target_ink_pixels"),
            "target_roi_erasure_ratio": raw.get("target_roi_erasure_ratio"),
            "outside_ink_mask_change_ratio": raw.get("outside_ink_mask_change_ratio"),
        }
        validate_metrics(metrics, "metrics file")
    else:
        metrics = {"available": False, "before_target_ink_pixels": None, "residual_target_ink_pixels": None,
                   "target_roi_erasure_ratio": None, "outside_ink_mask_change_ratio": None}
    evidence = {"before_mask_sha256": args.before_mask_sha256, "after_mask_sha256": args.after_mask_sha256,
                "board_mask_sha256": args.board_mask_sha256, "target_roi_mask_sha256": args.target_roi_mask_sha256,
                "video_id_or_redacted_path": args.video_id_or_redacted_path}
    record = {"schema_version": "1.0", "run_id": run_id, "trial_id": args.trial_id, "status": args.status,
              "started_at_utc": args.started_at_utc, "ended_at_utc": args.ended_at_utc,
              "duration_seconds": (ended - started).total_seconds(), "metrics": metrics,
              "evidence": evidence, "operator_notes": args.operator_notes}
    validate_trial(record, run_id)
    if args.output.name != f"{args.trial_id}.json":
        raise ValueError("output filename must be <trial_id>.json")
    output = ensure_artifacts_output(args.output, run_id)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
