#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then echo "Usage: $0 <ros-distro> <evaluation-run-id> <training-run-id>" >&2; exit 2; fi
if [[ ! "$2" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]]; then echo "Invalid evaluation run ID" >&2; exit 2; fi
if [[ ! "$3" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$ ]]; then echo "Invalid training run ID" >&2; exit 2; fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
Q_TABLE="$ROOT/runs/$3/q_table.json"
if [[ ! -f "$Q_TABLE" ]]; then echo "Training Q table not found: $Q_TABLE" >&2; exit 1; fi
source "/opt/ros/$1/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"
ros2 run qlearn_turtle_nav navigator --ros-args -p mode:=evaluate -p evaluation_trials:=10 -p seed:=7 -p q_table_path:="$Q_TABLE" -p q_table_source_run_id:="$3" -p output_directory:="$ROOT/runs" -p run_id:="$2"
