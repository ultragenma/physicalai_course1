"""Assess whiteboard erasure from binary masks without robot middleware.

Masks use 255 for pixels inside a region and 0 elsewhere.  The before/after
ink masks can come from a learned segmenter or a calibrated classical vision
pipeline; this evaluator deliberately keeps that choice outside the metric.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class ErasureMetrics:
    ink_before: int
    residual_ink: int
    erasure_ratio: float
    outside_ink_mask_change_ratio: float


def measure(before_ink, after_ink, board_mask, target_roi_mask) -> ErasureMetrics:
    """Measure target-ROI ink reduction and ink-mask changes outside the board.

    Inputs are same-shaped NumPy-like 2-D arrays. `before_ink`/`after_ink` are
    binary ink masks; `board_mask` marks the board; `target_roi_mask` encloses
    only the writing selected for removal.  The outside metric measures only
    segmentation-mask changes outside the board, not arbitrary image changes.
    """
    import numpy as np

    before = np.asarray(before_ink, dtype=bool)
    after = np.asarray(after_ink, dtype=bool)
    board = np.asarray(board_mask, dtype=bool)
    target_roi = np.asarray(target_roi_mask, dtype=bool)
    if before.ndim != 2 or before.size == 0:
        raise ValueError("masks must be non-empty 2-D arrays")
    if before.shape != after.shape or before.shape != board.shape or before.shape != target_roi.shape:
        raise ValueError("all masks must have identical shape")
    target = target_roi & board
    if not np.any(board) or not np.any(target):
        raise ValueError("board and target ROI masks must be non-empty")
    ink_before = int(np.count_nonzero(before & target))
    if ink_before == 0:
        raise ValueError("before_ink must contain at least one target pixel")
    residual = int(np.count_nonzero(after & target))
    erasure = 1.0 - residual / ink_before
    outside = ~board
    denominator = max(1, int(np.count_nonzero(outside)))
    outside_changes = int(np.count_nonzero((before ^ after) & outside))
    return ErasureMetrics(ink_before, residual, erasure, outside_changes / denominator)


def _read_mask(path: Path):
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("Install opencv-python to load PNG masks.") from exc
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image > 127


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure whiteboard erasure from PNG masks")
    parser.add_argument("--before-ink", type=Path, required=True)
    parser.add_argument("--after-ink", type=Path, required=True)
    parser.add_argument("--board-mask", type=Path, required=True)
    parser.add_argument("--target-roi-mask", type=Path, required=True)
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Write machine-readable metrics JSON. Store under ignored artifacts/.",
    )
    args = parser.parse_args()
    metrics = measure(
        _read_mask(args.before_ink),
        _read_mask(args.after_ink),
        _read_mask(args.board_mask),
        _read_mask(args.target_roi_mask),
    )
    payload = {
        "schema_version": "1.0",
        "metric_definition": {
            "target_roi_erasure_ratio": "1 - residual_target_ink_pixels / before_target_ink_pixels",
            "outside_ink_mask_change_ratio": "changed binary-ink-mask pixels outside board / all pixels outside board",
        },
        "before_target_ink_pixels": metrics.ink_before,
        "residual_target_ink_pixels": metrics.residual_ink,
        "target_roi_erasure_ratio": metrics.erasure_ratio,
        "outside_ink_mask_change_ratio": metrics.outside_ink_mask_change_ratio,
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
