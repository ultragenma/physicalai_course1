#!/usr/bin/env python3
"""Prepare an isolated CUDA-12.8 MolmoAct2 environment on the GPU workstation."""

import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--python", default="python3.12")
    ap.add_argument("--venv", type=Path, default=ROOT / ".venv-molmoact2")
    ap.add_argument("--check-only", action="store_true")
    ap.add_argument("--download-model", action="store_true")
    args = ap.parse_args()
    interpreter = args.venv / "bin/python"
    if not args.check_only:
        if not interpreter.is_file():
            subprocess.run([args.python, "-m", "venv", str(args.venv)], check=True)
        subprocess.run([str(interpreter), "-m", "pip", "install", "--upgrade", "pip"], check=True)
        subprocess.run([str(interpreter), "-m", "pip", "install", "torch==2.11.0", "torchvision==0.26.0",
                        "--index-url", "https://download.pytorch.org/whl/cu128"], check=True)
        subprocess.run([str(interpreter), "-m", "pip", "install", "-r", str(ROOT / "requirements-molmoact2-server.txt")], check=True)
    subprocess.run([str(interpreter), str(ROOT / "scripts/gpu_preflight.py")], cwd=ROOT, check=True)
    if args.download_model:
        lock = json.loads((ROOT / "configs/molmoact2/model.lock.json").read_text())
        subprocess.run([str(args.venv / "bin/hf"), "download", lock["model_id"], "--revision", lock["revision"],
                        "--local-dir", str(ROOT / "checkpoints/molmoact2-droid")], check=True)
    print("Ready. Start the target selector with .venv-molmoact2/bin/python -m wb_eraser.vlm_server")


if __name__ == "__main__":
    main()
