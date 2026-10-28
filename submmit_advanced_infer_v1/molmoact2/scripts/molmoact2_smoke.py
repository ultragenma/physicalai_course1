#!/usr/bin/env python3
"""GPU-only smoke inference using the official DROID reference observation."""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from wb_eraser.molmoact2_contract import Profile, checkpoint_schema
from wb_eraser.molmoact2_server import MolmoAct2Backend


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", type=Path, default=ROOT / "checkpoints/molmoact2-droid")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--dtype", choices=["bfloat16", "float16", "float32"], default="bfloat16")
    ap.add_argument("--out", type=Path, default=ROOT / "runs/molmoact2/gpu_smoke.json")
    args = ap.parse_args()
    checkpoint = args.checkpoint.resolve()
    schema = checkpoint_schema(checkpoint, Profile.load(ROOT / "configs/molmoact2/droid_probe.json"))
    report = {"complete": False, "input_source": "official MolmoAct2-DROID model-card reference observation",
              "hardware_robot_commands_sent": 0, "aiworker_policy_performance_measured": False}
    try:
        backend = MolmoAct2Backend(checkpoint, schema, args.device, args.dtype)
        ext = np.array(Image.open(checkpoint / "assets/sample_exterior_1_left_rgb.png").convert("RGB"))
        wrist = np.array(Image.open(checkpoint / "assets/sample_wrist_left_rgb.png").convert("RGB"))
        state = np.array([-0.12726949, -0.30641943, 0.09134164, -2.4143615,
                          -0.26460838, 2.068765, 0.123698, 0.0], np.float32)
        start = time.monotonic()
        actions = backend.predict([ext, ext, wrist], state,
                                  "Put the black objects into the drawer and close the drawer.")
        report.update(complete=True, action_shape=list(actions.shape), actions=actions.tolist(),
                      inference_s=time.monotonic() - start, action_names=schema["action_names"])
    except Exception as exc:
        report["error"] = str(exc)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"GPU smoke: complete={report['complete']}; {args.out}")
    if "error" in report:
        print(report["error"])
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
