#!/usr/bin/env python3
"""Record the ROS 2 graph for an AIWorker interface review.

Read-only inspection. It launches no nodes and publishes nothing. The generated
JSON is potentially sensitive; store it under `artifacts/` and do not publish it
without removing internal names, hosts, or paths.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def run(*args: str) -> dict[str, object]:
    result = subprocess.run(args, text=True, capture_output=True, check=False)
    return {"command": list(args), "returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "warning": "Read-only graph snapshot. Redact before public sharing.",
        "topic_list_t": run("ros2", "topic", "list", "-t"),
        "action_list_t": run("ros2", "action", "list", "-t"),
        "service_list_t": run("ros2", "service", "list", "-t"),
        "node_list": run("ros2", "node", "list"),
    }
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    failed = any(payload[key]["returncode"] != 0 for key in ("topic_list_t", "action_list_t", "service_list_t", "node_list"))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
