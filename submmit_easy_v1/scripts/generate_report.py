#!/usr/bin/env python3
"""Create a report-ready Markdown factsheet from navigator JSON run records."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def metric_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def markdown_cell(value):
    return str(value).replace("|", "\\|").replace("\r", " ").replace("\n", " ").strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--train-run", type=Path, required=True)
    parser.add_argument("--eval-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    train = read_json(args.train_run / "run_metadata.json")
    evaluation = read_json(args.eval_run / "run_metadata.json")
    training_summary = read_json(args.train_run / "training_summary.json")
    summary = read_json(args.eval_run / "evaluation_summary.json")
    if train["mode"] != "train" or evaluation["mode"] != "evaluate":
        raise ValueError("--train-run must be train and --eval-run must be evaluate")

    mismatches = []
    if train["seed"] != evaluation["seed"]:
        mismatches.append("seed differs")
    if train["target"] != evaluation["target"]:
        mismatches.append("target coordinates differ")
    if train["start_pose"] != evaluation["start_pose"]:
        mismatches.append("start pose differs")
    for key in ("ros_distro", "rclpy_version", "python_version", "control_period_s", "max_steps_per_episode"):
        if train.get(key) != evaluation.get(key):
            mismatches.append(f"{key} differs")
    if mismatches:
        raise ValueError("Refusing to combine runs with mismatched conditions: " + ", ".join(mismatches))
    train_run_id = train["run_id"]
    eval_run_id = evaluation["run_id"]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,63}", train_run_id):
        raise ValueError("training run_id is not a safe run identifier")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,63}", eval_run_id):
        raise ValueError("evaluation run_id is not a safe run identifier")
    if training_summary.get("run_id") != train_run_id or summary.get("run_id") != eval_run_id:
        raise ValueError("summary run_id does not match its run metadata")
    train_q_hash = training_summary.get("q_table_sha256")
    eval_q_hash = summary.get("q_table_sha256")
    if not isinstance(train_q_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", train_q_hash):
        raise ValueError("training summary has no valid Q-table SHA-256")
    if not isinstance(eval_q_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", eval_q_hash):
        raise ValueError("evaluation summary has no valid Q-table SHA-256")
    if train_q_hash != eval_q_hash:
        raise ValueError("evaluation used a different Q-table than the selected training run")
    if train.get("q_table_sha256") != train_q_hash:
        raise ValueError("training metadata and summary disagree on Q-table SHA-256")
    if evaluation.get("q_table_sha256") != eval_q_hash:
        raise ValueError("evaluation metadata and summary disagree on Q-table SHA-256")
    if evaluation.get("q_table_source_run_id") != train_run_id:
        raise ValueError("evaluation metadata does not identify the selected training run as Q-table source")
    if training_summary.get("q_table_file") != "q_table.json" or train.get("q_table_file") != "q_table.json":
        raise ValueError("training run does not identify its immutable q_table.json snapshot")
    if summary.get("q_table_file") != "q_table.json" or evaluation.get("q_table_file") != "q_table.json":
        raise ValueError("evaluation run does not identify the expected q_table.json file")
    q_table_bytes = (args.train_run / "q_table.json").read_bytes()
    if hashlib.sha256(q_table_bytes).hexdigest() != train_q_hash:
        raise ValueError("training run's saved Q-table file does not match its recorded SHA-256")
    if summary.get("trial_results_file") != "evaluation_trials.jsonl":
        raise ValueError("evaluation summary references an unexpected trial-results filename")
    ros_distro = markdown_cell(evaluation["ros_distro"])
    rclpy_version = markdown_cell(evaluation["rclpy_version"])
    python_version = markdown_cell(evaluation["python_version"])
    target_x = metric_number(evaluation["target"]["x"], "target.x")
    target_y = metric_number(evaluation["target"]["y"], "target.y")
    start_x = metric_number(evaluation["start_pose"]["x"], "start_pose.x")
    start_y = metric_number(evaluation["start_pose"]["y"], "start_pose.y")
    start_theta = metric_number(evaluation["start_pose"]["theta"], "start_pose.theta")
    episodes_completed = training_summary["episodes_completed"]
    training_successes = training_summary["successes"]
    evaluation_trials = summary["trials"]
    evaluation_successes = summary["successes"]
    if any(isinstance(value, bool) or not isinstance(value, int) for value in (episodes_completed, training_successes, evaluation_trials, evaluation_successes)):
        raise ValueError("episode/trial counts must be integers")
    if episodes_completed <= 0 or training_successes < 0 or training_successes > episodes_completed:
        raise ValueError("training summary counts are inconsistent")
    if evaluation_trials <= 0 or evaluation_successes < 0 or evaluation_successes > evaluation_trials:
        raise ValueError("evaluation summary counts are inconsistent")
    if isinstance(train.get("seed"), bool) or not isinstance(train.get("seed"), int):
        raise ValueError("seed must be an integer")
    if isinstance(train.get("max_steps_per_episode"), bool) or not isinstance(train.get("max_steps_per_episode"), int):
        raise ValueError("max_steps_per_episode must be an integer")
    if train["max_steps_per_episode"] <= 0 or metric_number(train["control_period_s"], "control_period_s") <= 0:
        raise ValueError("training control parameters must be positive")
    training_success_rate = metric_number(training_summary["success_rate"], "training success_rate")
    evaluation_success_rate = metric_number(summary["success_rate"], "evaluation success_rate")
    if not math.isclose(training_success_rate, training_successes / episodes_completed, abs_tol=1e-9):
        raise ValueError("training success_rate does not match its counts")
    if not math.isclose(evaluation_success_rate, evaluation_successes / evaluation_trials, abs_tol=1e-9):
        raise ValueError("evaluation success_rate does not match its counts")
    if training_summary.get("episode_results_file") != "training_episodes.jsonl":
        raise ValueError("training summary references an unexpected episode-results filename")
    trial_results_path = "runs/" + eval_run_id + "/evaluation_trials.jsonl"
    mean_elapsed_s = metric_number(summary["mean_elapsed_s"], "mean_elapsed_s")
    if mean_elapsed_s < 0:
        raise ValueError("mean_elapsed_s must be non-negative")

    report = f"""# 実行記録から生成したレポート追記

このファイルは実行ログから機械的に生成した。動画URL、GitHub URL、定性的な考察は手動で追記する。

| 項目 | 値 |
| --- | --- |
| 学習 run ID | {train_run_id} |
| 評価 run ID | {eval_run_id} |
| ROS 2ディストリビューション / rclpy版 | {ros_distro} / {rclpy_version} |
| Python版 | {python_version} |
| 目標座標 | ({target_x}, {target_y}) |
| 開始姿勢 | ({start_x}, {start_y}, {start_theta}) |
| 学習完了エピソード数 | {episodes_completed} |
| 学習中の到達数 / 成功率 | {training_successes} / {training_success_rate:.4f} ({training_success_rate:.1%}) |
| 乱数seed | {train["seed"]} |
| 制御周期 / 1エピソード最大ステップ | {metric_number(train["control_period_s"], "control_period_s")} 秒 / {train["max_steps_per_episode"]} |
| 評価試行数 | {evaluation_trials} |
| 評価到達数 / 成功率 | {evaluation_successes} / {evaluation_success_rate:.4f} ({evaluation_success_rate:.1%}) |
| 平均評価所要時間 | {mean_elapsed_s:.3f} 秒 |
| 使用Q表のSHA-256 | `{train_q_hash}` |
| 試行の詳細 | {trial_results_path} |

この成功率はturtlesim上の設定した開始位置・目標位置に対する値であり、実機性能を表さない。"""
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(report + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
