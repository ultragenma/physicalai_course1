"""Learn a small goal-reaching policy from turtlesim pose observations."""

import json
import hashlib
import math
import os
import re
import random
import sys
import time
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute


Action = int
State = Tuple[int, int]
ACTIONS = (0, 1, 2)  # forward, turn left, turn right


class QLearnTurtleNavigator(Node):
    """Discrete Q-learning controller; no target-heading rule selects actions."""

    def __init__(self) -> None:
        super().__init__("qlearn_turtle_navigator")
        self.declare_parameter("mode", "train")
        self.declare_parameter("episodes", 60)
        self.declare_parameter("control_period_s", 0.05)
        self.declare_parameter("max_steps_per_episode", 220)
        self.declare_parameter("target_x", 8.5)
        self.declare_parameter("target_y", 8.5)
        self.declare_parameter("seed", 7)
        self.declare_parameter("q_table_path", "q_table.json")
        self.declare_parameter("output_directory", "runs")
        self.declare_parameter("run_id", "")
        self.declare_parameter("q_table_source_run_id", "")
        self.declare_parameter("evaluation_trials", 1)
        self.mode = str(self.get_parameter("mode").value)
        self.episodes = int(self.get_parameter("episodes").value)
        self.control_period_s = float(self.get_parameter("control_period_s").value)
        self.max_steps_per_episode = int(self.get_parameter("max_steps_per_episode").value)
        self.target = (float(self.get_parameter("target_x").value), float(self.get_parameter("target_y").value))
        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.q_path = Path(str(self.get_parameter("q_table_path").value))
        self.output_directory = Path(str(self.get_parameter("output_directory").value))
        self.run_id = str(self.get_parameter("run_id").value).strip()
        if not self.run_id:
            raise ValueError("run_id is required to create reproducible machine-readable records")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,63}", self.run_id):
            raise ValueError("run_id must be 3-64 letters, digits, underscores, or hyphens")
        self.run_directory = self.output_directory / self.run_id
        self.q_table_source_run_id = str(self.get_parameter("q_table_source_run_id").value).strip()
        self.evaluation_trials = int(self.get_parameter("evaluation_trials").value)
        if self.evaluation_trials < 1:
            raise ValueError("evaluation_trials must be positive")
        self.pose = None
        self.exit_code = 0
        self.previous_distance = None
        self.step_count = 0
        self.episode = 0
        self.q: Dict[str, list[float]] = {}
        self.current_state = None
        self.current_action = None
        self.resetting = False
        self.awaiting_reset_pose = False
        self.reset_pose_deadline = None
        self.q_table_sha256 = None
        self.created_at_utc = datetime.now(timezone.utc).isoformat()
        self.waiting_for_reset_service = False
        self.episode_started_at = None
        self.evaluation_results = []
        self.training_results = []
        self.mode = self.mode.lower()
        if self.mode not in {"train", "evaluate"}:
            raise ValueError("mode must be 'train' or 'evaluate'")
        if self.episodes <= 0 or not math.isfinite(self.control_period_s) or self.control_period_s <= 0 or self.max_steps_per_episode <= 0:
            raise ValueError("episodes, control_period_s, and max_steps_per_episode must be positive")
        if not all(0.5 <= coordinate <= 10.5 for coordinate in self.target):
            raise ValueError("target_x and target_y must stay inside the safe turtlesim range [0.5, 10.5]")
        if self.mode == "evaluate":
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{2,63}", self.q_table_source_run_id):
                raise ValueError("evaluation requires q_table_source_run_id matching the training run")
            self._load_q()
        else:
            self.q_table_source_run_id = self.run_id
        self.run_directory.mkdir(parents=True, exist_ok=False)
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self._pose_callback, 10)
        self.reset_client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.timer = self.create_timer(self.control_period_s, self._control_step)
        self.get_logger().info(
            f"mode={self.mode}, target=({self.target[0]:.2f}, {self.target[1]:.2f}), q_table={self.q_path.name}"
        )
        self._write_metadata()
        self._reset_episode()

    def _pose_callback(self, pose: Pose) -> None:
        self.pose = pose
        if self.awaiting_reset_pose:
            dx = pose.x - 2.0
            dy = pose.y - 2.0
            heading_error = self._wrap(pose.theta)
            if math.hypot(dx, dy) > 0.10 or abs(heading_error) > 0.20:
                return
            self.awaiting_reset_pose = False
            self.reset_pose_deadline = None
            self.previous_distance = None
            self.step_count = 0
            self.current_state = None
            self.current_action = None
            self.episode_started_at = time.monotonic()

    def _distance_and_error(self) -> Tuple[float, float]:
        dx = self.target[0] - self.pose.x
        dy = self.target[1] - self.pose.y
        return math.hypot(dx, dy), self._wrap(math.atan2(dy, dx) - self.pose.theta)

    @staticmethod
    def _wrap(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _state(self) -> State:
        distance, heading_error = self._distance_and_error()
        distance_bin = min(5, int(distance / 2.0))
        angle_bin = int((heading_error + math.pi) / (2.0 * math.pi / 8.0)) % 8
        return distance_bin, angle_bin

    @staticmethod
    def _key(state: State) -> str:
        return f"{state[0]}:{state[1]}"

    def _values(self, state: State) -> list[float]:
        return self.q.setdefault(self._key(state), [0.0, 0.0, 0.0])

    def _choose_action(self, state: State) -> Action:
        if self.mode == "train":
            epsilon = max(0.05, 0.45 * (1.0 - self.episode / max(1, self.episodes)))
            if self.rng.random() < epsilon:
                return self.rng.choice(ACTIONS)
        values = self._values(state)
        best = max(values)
        return self.rng.choice([a for a, value in enumerate(values) if value == best])

    def _command(self, action: Action) -> None:
        command = Twist()
        if action == 0:
            command.linear.x = 1.2
        elif action == 1:
            command.angular.z = 1.8
        else:
            command.angular.z = -1.8
        self.publisher.publish(command)

    def _stop(self) -> None:
        self.publisher.publish(Twist())

    def _reset_episode(self) -> None:
        """Start every training episode from a reproducible turtlesim pose."""
        if not self.reset_client.service_is_ready():
            if not self.waiting_for_reset_service:
                self.get_logger().info("Waiting for /turtle1/teleport_absolute service")
                self.waiting_for_reset_service = True
            return
        self.waiting_for_reset_service = False
        request = TeleportAbsolute.Request()
        request.x, request.y, request.theta = 2.0, 2.0, 0.0
        self.resetting = True
        future = self.reset_client.call_async(request)
        future.add_done_callback(self._reset_done)

    def _reset_done(self, future) -> None:
        try:
            future.result()
            # The service response may arrive before the telemetry reflects
            # the teleport. Wait for a pose near the requested reset pose.
            self.awaiting_reset_pose = True
            self.reset_pose_deadline = time.monotonic() + 5.0
        except Exception as error:  # ROS client failures must be visible to the operator.
            self.exit_code = 1
            self._stop()
            self.get_logger().fatal(f"Episode reset failed; stopping the node: {error}")
            rclpy.shutdown()
        finally:
            self.resetting = False

    def _reward(self, distance: float) -> Tuple[float, bool]:
        reached = distance < 0.45
        near_wall = min(self.pose.x, self.pose.y, 11.0 - self.pose.x, 11.0 - self.pose.y) < 0.45
        progress = 0.0 if self.previous_distance is None else self.previous_distance - distance
        reward = progress * 8.0 - 0.05
        if near_wall:
            reward -= 2.0
        if reached:
            reward += 30.0
        return reward, reached

    def _control_step(self) -> None:
        if self.pose is None or self.resetting:
            return
        if self.awaiting_reset_pose:
            if self.reset_pose_deadline is not None and time.monotonic() > self.reset_pose_deadline:
                self.exit_code = 1
                self._stop()
                self.get_logger().fatal("Timed out waiting for the reset pose on /turtle1/pose; stopping.")
                rclpy.shutdown()
            return
        if self.waiting_for_reset_service or not self.reset_client.service_is_ready():
            self._reset_episode()
            return
        state = self._state()
        distance, _ = self._distance_and_error()
        reward, reached = self._reward(distance)
        timed_out = self.step_count >= self.max_steps_per_episode
        if self.current_state is not None and self.mode == "train":
            alpha, gamma = 0.25, 0.92
            old = self._values(self.current_state)[self.current_action]
            target = reward + (0.0 if reached or timed_out else gamma * max(self._values(state)))
            self._values(self.current_state)[self.current_action] = old + alpha * (target - old)
        if reached or timed_out:
            self._stop()
            if self.mode == "evaluate":
                trial = len(self.evaluation_results) + 1
                result = self._result_record("trial", trial, reached, distance)
                self.evaluation_results.append(result)
                self._append_jsonl("evaluation_trials.jsonl", result)
                self.get_logger().info(f"trial={trial}/{self.evaluation_trials} " + ("GOAL REACHED" if reached else "EVALUATION TIMEOUT"))
                if trial >= self.evaluation_trials:
                    self._write_evaluation_summary()
                    rclpy.shutdown()
                else:
                    self._reset_episode()
                return
            self.episode += 1
            result = self._result_record("episode", self.episode, reached, distance)
            self.training_results.append(result)
            self._append_jsonl("training_episodes.jsonl", result)
            self.get_logger().info(f"episode={self.episode}/{self.episodes} success={reached} steps={self.step_count}")
            if self.episode >= self.episodes:
                self._save_q()
                self._write_metadata()
                self._write_training_summary()
                self.get_logger().info("TRAINING COMPLETE; use this run's q_table.json and run ID for evaluation.")
                rclpy.shutdown()
            else:
                self._reset_episode()
            return
        self.current_state = state
        self.current_action = self._choose_action(state)
        self._command(self.current_action)
        self.previous_distance = distance
        self.step_count += 1

    def _save_q(self) -> None:
        content = json.dumps(self.q, indent=2).encode("utf-8")
        self.q_path.write_bytes(content)
        snapshot = self.run_directory / "q_table.json"
        if self.q_path.resolve() != snapshot.resolve():
            snapshot.write_bytes(content)
        self.q_table_sha256 = hashlib.sha256(content).hexdigest()

    def _load_q(self) -> None:
        if not self.q_path.exists():
            raise FileNotFoundError(f"Q table not found: {self.q_path}. Run mode:=train first.")
        content = self.q_path.read_bytes()
        self.q = json.loads(content.decode("utf-8"))
        self.q_table_sha256 = hashlib.sha256(content).hexdigest()

    def _result_record(self, label, index, reached, distance):
        return {
            label: index,
            "success": reached,
            "steps": self.step_count,
            "elapsed_s": round(time.monotonic() - self.episode_started_at, 3),
            "final_distance_m": round(distance, 4),
        }

    def _append_jsonl(self, name, record):
        with (self.run_directory / name).open("a", encoding="utf-8") as output:
            output.write(json.dumps(record, sort_keys=True) + "\n")

    def _write_metadata(self):
        try:
            rclpy_version = version("rclpy")
        except PackageNotFoundError:
            rclpy_version = "unknown"
        metadata = {
            "schema_version": 1,
            "created_at_utc": self.created_at_utc,
            "mode": self.mode,
            "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
            "rclpy_version": rclpy_version,
            "python_version": sys.version.split()[0],
            "seed": int(self.get_parameter("seed").value),
            "run_id": self.run_id,
            "target": {"x": self.target[0], "y": self.target[1]},
            "start_pose": {"x": 2.0, "y": 2.0, "theta": 0.0},
            "control_period_s": self.control_period_s,
            "max_steps_per_episode": self.max_steps_per_episode,
            "training_episodes_requested": self.episodes if self.mode == "train" else None,
            "evaluation_trials_requested": self.evaluation_trials if self.mode == "evaluate" else None,
            "q_table_file": "q_table.json" if self.mode == "train" else self.q_path.name,
            "q_table_source_run_id": self.q_table_source_run_id,
            "q_table_sha256": self.q_table_sha256,
        }
        (self.run_directory / "run_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    def _write_training_summary(self):
        successes = sum(record["success"] for record in self.training_results)
        summary = {
            "schema_version": 1,
            "run_id": self.run_id,
            "episodes_completed": len(self.training_results),
            "successes": successes,
            "success_rate": successes / len(self.training_results),
            "q_table_file": "q_table.json",
            "q_table_sha256": self.q_table_sha256,
            "episode_results_file": "training_episodes.jsonl",
        }
        (self.run_directory / "training_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    def _write_evaluation_summary(self):
        successes = sum(record["success"] for record in self.evaluation_results)
        elapsed = [record["elapsed_s"] for record in self.evaluation_results]
        summary = {
            "schema_version": 1,
            "run_id": self.run_id,
            "trials": len(self.evaluation_results),
            "successes": successes,
            "success_rate": successes / len(self.evaluation_results),
            "mean_elapsed_s": sum(elapsed) / len(elapsed),
            "q_table_file": self.q_path.name,
            "q_table_sha256": self.q_table_sha256,
            "trial_results_file": "evaluation_trials.jsonl",
        }
        (self.run_directory / "evaluation_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = QLearnTurtleNavigator()
        rclpy.spin(node)
        return node.exit_code
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
