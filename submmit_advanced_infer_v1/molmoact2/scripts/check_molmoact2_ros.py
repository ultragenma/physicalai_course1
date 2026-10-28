#!/usr/bin/env python3
"""Run the ROS/HTTP fixture in an isolated, disposable local test container."""

import argparse
import os
from pathlib import Path
import subprocess


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image", default="physicalai-turtlesim:jazzy-v1")
    ap.add_argument("--fixture", choices=["policy", "hybrid"], default="policy")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    fixture = "tests/ros_hybrid_fixture.py" if args.fixture == "hybrid" else "tests/ros_fixture_molmoact2.py"
    overlay = 'source /root/ros2_ws/install/setup.bash; ' if args.fixture == "hybrid" else ''
    bootstrap = ('source /opt/ros/jazzy/setup.bash; ' + overlay +
                 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1; '
                 'unset ZENOH_CONFIG_OVERRIDE; export PYTHONPATH=/task:"${PYTHONPATH:-}"; '
                 'python3 ' + fixture)
    user = "0:0" if args.fixture == "hybrid" else f"{os.getuid()}:{os.getgid()}"
    logdir = "/task/runs/hybrid/ros_fixture/ros_logs" if args.fixture == "hybrid" else "/task/runs/molmoact2/ros_fixture/ros_logs"
    subprocess.run(["docker", "run", "--rm", "--network", "none", "--ipc", "private",
                    "--user", user,
                    "--mount", f"type=bind,src={root},dst=/task", "--workdir", "/task",
                    "--env", "ROS_LOG_DIR=" + logdir,
                    "--env", f"WB_TEST_UID={os.getuid()}", "--env", f"WB_TEST_GID={os.getgid()}",
                    "--entrypoint", "/bin/bash", args.image, "-c", bootstrap], check=True, timeout=100)


if __name__ == "__main__":
    main()
