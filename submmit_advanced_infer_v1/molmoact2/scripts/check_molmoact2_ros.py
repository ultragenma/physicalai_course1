#!/usr/bin/env python3
"""Run the ROS/HTTP fixture in an isolated, disposable local test container."""

import argparse
import os
from pathlib import Path
import subprocess


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image", default="physicalai-turtlesim:jazzy-v1")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    bootstrap = ('source /opt/ros/jazzy/setup.bash; '
                 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1; '
                 'unset ZENOH_CONFIG_OVERRIDE; export PYTHONPATH=/task:"${PYTHONPATH:-}"; '
                 'python3 tests/ros_fixture_molmoact2.py')
    subprocess.run(["docker", "run", "--rm", "--network", "none", "--ipc", "private",
                    "--user", f"{os.getuid()}:{os.getgid()}",
                    "--mount", f"type=bind,src={root},dst=/task", "--workdir", "/task",
                    "--env", "ROS_LOG_DIR=/task/runs/molmoact2/ros_fixture/ros_logs",
                    "--entrypoint", "/bin/bash", args.image, "-c", bootstrap], check=True, timeout=100)


if __name__ == "__main__":
    main()
