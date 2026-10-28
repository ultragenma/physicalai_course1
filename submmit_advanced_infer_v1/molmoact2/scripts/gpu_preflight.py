#!/usr/bin/env python3
"""Check CUDA/PyTorch compatibility, including RTX 5090, without loading a model."""

import argparse
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--out", type=Path, default=Path("runs/hybrid/gpu_preflight.json"))
    args = ap.parse_args()
    result = {"complete": False, "model_forward_executed": False}
    try:
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable; check the NVIDIA driver and CUDA-enabled PyTorch")
        device = torch.device(args.device)
        cap = torch.cuda.get_device_capability(device)
        arches = torch.cuda.get_arch_list()
        result.update(torch_version=torch.__version__, cuda_version=torch.version.cuda,
                      device_name=torch.cuda.get_device_name(device), capability=list(cap), arches=arches,
                      total_vram_bytes=torch.cuda.get_device_properties(device).total_memory,
                      bfloat16_supported=torch.cuda.is_bf16_supported())
        if cap == (12, 0) and "sm_120" not in arches:
            raise RuntimeError("RTX 5090 needs an sm_120 PyTorch build; install the CUDA 12.8 wheel")
        tensor = torch.ones((16, 16), device=device)
        assert float((tensor @ tensor)[0, 0]) == 16.
        torch.cuda.synchronize(device)
        result.update(complete=True, cuda_kernel_check=True)
    except Exception as exc:
        result["error"] = str(exc)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
