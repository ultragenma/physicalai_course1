#!/usr/bin/env python3
"""Capture sensor metadata in the existing ai_worker container, without syncing files.

The standalone inspector is passed on stdin. The existing shell's ROS/Zenoh
settings are retained; no controller, robot bringup, router, or policy starts.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

BEGIN = "WHITEBOARD_OBSERVATIONS_JSON_BEGIN"
END = "WHITEBOARD_OBSERVATIONS_JSON_END"
BOOTSTRAP = ("source /opt/ros/jazzy/setup.bash; "
             "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi; "
             'exec "$@"')


def parse_output(output: str) -> dict:
    if BEGIN not in output or END not in output:
        raise ValueError("ROS inspector did not return a complete metadata block; inspect the local log")
    return json.loads(output.split(BEGIN, 1)[1].split(END, 1)[0])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--container", default="ai_worker")
    ap.add_argument("--out", type=Path, default=Path("runs/connection/observations.json"))
    ap.add_argument("--camera", action="append")
    ap.add_argument("--joint-state-topic", default="/joint_states")
    ap.add_argument("--timeout", type=float, default=12)
    ap.add_argument("--samples", type=int, default=3)
    ap.add_argument("--connection-env", type=Path,
                    help="private JSON with verified RMW_IMPLEMENTATION/ROS_DOMAIN_ID/ZENOH_CONFIG_OVERRIDE")
    args = ap.parse_args(argv)
    source = Path(__file__).with_name("inspect_observations.py").read_text(encoding="utf-8")
    allowed = ("RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ZENOH_CONFIG_OVERRIDE")
    overrides = {key: os.environ[key] for key in allowed if key in os.environ}
    if args.connection_env:
        configured = json.loads(args.connection_env.read_text(encoding="utf-8"))
        if (not isinstance(configured, dict) or set(configured) - set(allowed)
                or any(not isinstance(v, str) for v in configured.values())):
            ap.error("connection-env must be an object of string-valued ROS connection settings only")
        overrides.update(configured)
    command = ["docker", "exec", "-i", args.container, "bash", "-ic", BOOTSTRAP, "wb_readonly", "env"]
    command += [key + "=" + value for key, value in overrides.items()]
    command += ["python3", "-", "--joint-state-topic", args.joint_state_topic, "--timeout", str(args.timeout),
               "--samples", str(args.samples)]
    for camera in args.camera or []:
        command += ["--camera", camera]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    try:
        process = subprocess.run(command, input=source, text=True, capture_output=True, timeout=args.timeout + 20)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"Capture failed: {exc}", file=sys.stderr)
        return 2
    args.out.with_suffix(".log").write_text(process.stdout + "\n" + process.stderr, encoding="utf-8")
    try:
        result = parse_output(process.stdout)
    except ValueError as exc:
        print(f"{exc}: {args.out.with_suffix('.log')}", file=sys.stderr)
        return 2
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"Metadata: {args.out}; complete={result['complete']}; "
          f"topics={len(result['graph']['topics'])}; actuator_commands_sent=0")
    for name, stream in {**result["cameras"], "joint_state": result["joint_state"]}.items():
        print(f"  {name}: {stream['count']} samples")
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
