#!/usr/bin/env python3
"""Create a privacy-gated report factsheet from reviewed metadata and trials."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import mean

from record_validation import ensure_artifacts_output, ensure_artifacts_run_path, validate_metadata, validate_trial


def load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def fmt(value: float | None, digits: int = 3) -> str:
    return "未計測" if value is None else f"{value:.{digits}f}"


def public_text(value: object) -> str:
    """Keep reviewed free-form metadata from breaking Markdown structure."""
    return str(value).replace("|", "\\|").replace("\r", " ").replace("\n", " ").strip()


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate privacy-gated Markdown report facts from simulation trial records")
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--trials-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary-csv", type=Path, help="Optional sanitized per-trial summary CSV")
    parser.add_argument(
        "--include-reviewed-environment", action="store_true",
        help="Export technical metadata only after every exported environment field has been anonymized and approved",
    )
    args = parser.parse_args()

    metadata = validate_metadata(load(args.metadata), require_public_review=True)
    run_id = metadata["run_id"]
    trials_dir = ensure_artifacts_run_path(args.trials_dir, run_id, "trials directory")
    trial_paths = sorted(trials_dir.glob("*.json"))
    trials = [validate_trial(load(path), run_id) for path in trial_paths]
    if not trials:
        raise ValueError("no trial records found; create a trial record for every attempted run")
    trial_ids = [trial["trial_id"] for trial in trials]
    if len(trial_ids) != len(set(trial_ids)):
        raise ValueError("trial_id values must be unique within a run")

    statuses: dict[str, int] = {}
    measured = []
    invalid_measured_count = 0
    for trial in trials:
        statuses[trial["status"]] = statuses.get(trial["status"], 0) + 1
        if trial["metrics"]["available"]:
            if trial["status"] == "invalid":
                invalid_measured_count += 1
            else:
                measured.append(trial["metrics"])
    erasures = [entry["target_roi_erasure_ratio"] for entry in measured]
    outside = [entry["outside_ink_mask_change_ratio"] for entry in measured]
    execution = metadata["execution"]
    software = metadata["software"]
    robot = metadata["robot"]
    gpu = software["gpu"]
    scene_revision = execution.get("scene_revision") or "未記録"
    lines = [
        "# レポートへ転記する公開確認済みファクト（自動生成）", "",
        "> 生成には個人情報・社内ネットワーク情報・機密情報の3項目すべてを人が確認し、metadata.jsonでfalseにする必要がある。operator_notes、動画参照、マスク参照、画像内容は出力しない。", "",
        "## 実験環境", "",
        f"- 人手確認済みの公開用要約: {public_text(metadata['privacy_review']['public_safe_summary'])}",
        f"- run_id: `{run_id}`", f"- 実行種別: {public_text(execution['kind'])}",
    ]
    if args.include_reviewed_environment:
        lines += [
            f"- シミュレータ: {public_text(execution['simulator'])}",
            f"- asset: {public_text(execution['asset_source'])} @ {public_text(execution['asset_revision'])} / {public_text(execution['asset_variant'])}",
            f"- scene revision: {public_text(scene_revision)}", f"- AIWorker型式: {public_text(robot['model'])}",
            f"- ROS: {public_text(software['ros_distro'])}; robot software: {public_text(software['robot_software_version'])}",
            f"- policy: {public_text(software['policy_runtime'])} @ {public_text(software['policy_runtime_revision'])}; model: {public_text(software['policy_model'])}; checkpoint: {public_text(software['checkpoint_id'])}",
            f"- GPU: {public_text(gpu['name'])} / driver {public_text(gpu['driver_version'])} / {gpu.get('vram_gb') if gpu.get('vram_gb') is not None else '未記録'} GB",
        ]
    else:
        lines.append("- 詳細な環境識別子は既定では出力しない。公開承認後に --include-reviewed-environment を指定できる。")
    lines += [
        "", "## 試行集計", "",
        f"- 全試行数: {len(trials)}", f"- 状態別: {', '.join(f'{key}={value}' for key, value in sorted(statuses.items())) or '記録なし'}",
        f"- 平均指標の対象試行数: {len(measured)} (metricsあり、invalidを除外。failed/abortedの計測値は含む)",
        f"- invalidのため平均から除外した計測済み試行数: {invalid_measured_count}",
        f"- 平均対象ROI消去率: {fmt(mean(erasures) if erasures else None)} (小数比; 0.90 = 90%)",
        f"- 平均白板外筆跡マスク変化率: {fmt(mean(outside) if outside else None, 6)} (小数比)", "", "## 試行別記録", "",
        "| trial_id | status | duration_s | target ROI erasure | outside ink-mask change |", "| --- | --- | ---: | ---: | ---: |",
    ]
    for trial in trials:
        metrics = trial["metrics"]
        lines.append(
            f"| {trial['trial_id']} | {trial['status']} | {trial['duration_seconds']:.3f} | "
            f"{fmt(metrics['target_roi_erasure_ratio'])} | {fmt(metrics['outside_ink_mask_change_ratio'], 6)} |"
        )
    lines += [
        "", "## 提出前に手動で埋める項目", "", "- 公開GitHub URL、YouTube URL、動画の再生速度",
        "- 成功定義と成功数（status=completedを成功扱いにしない）",
        "- 失敗例、介入、安全停止、未実装・未検証項目",
        "- 公開前にPDF・動画・公開リポジトリを再確認する", "",
    ]

    output = ensure_artifacts_output(args.output, run_id)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")
    if args.summary_csv:
        summary_csv = ensure_artifacts_output(args.summary_csv, run_id)
        summary_csv.parent.mkdir(parents=True, exist_ok=True)
        with summary_csv.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=("run_id", "trial_id", "status", "included_in_metric_averages", "duration_seconds", "target_roi_erasure_ratio", "outside_ink_mask_change_ratio"))
            writer.writeheader()
            for trial in trials:
                metrics = trial["metrics"]
                writer.writerow({"run_id": run_id, "trial_id": trial["trial_id"], "status": trial["status"],
                                 "included_in_metric_averages": metrics["available"] and trial["status"] != "invalid",
                                 "duration_seconds": trial["duration_seconds"],
                                 "target_roi_erasure_ratio": metrics["target_roi_erasure_ratio"],
                                 "outside_ink_mask_change_ratio": metrics["outside_ink_mask_change_ratio"]})


if __name__ == "__main__":
    main()
