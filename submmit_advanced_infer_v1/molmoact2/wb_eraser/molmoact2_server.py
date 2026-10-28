"""Separate-GPU MolmoAct2 HTTP inference service; never connects to ROS."""

from __future__ import annotations

import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
import time

import numpy as np

from .molmoact2_contract import MAX_BODY, Profile, check_actions, checkpoint_schema, parse_request


def inspect_weights(checkpoint):
    index_path = checkpoint / "model.safetensors.index.json"
    if index_path.is_file():
        index = json.loads(index_path.read_text())
        names = sorted(set(index["weight_map"].values()))
        total = int(index.get("metadata", {}).get("total_size", 1))
    else:
        names, total = ["model.safetensors"], 1
    if any(Path(n).name != n for n in names):
        raise ValueError("checkpoint weight index contains unsafe paths")
    missing = [name for name in names if not (checkpoint / name).is_file()]
    size = sum((checkpoint / name).stat().st_size for name in names if (checkpoint / name).is_file())
    required = ["config.json", "norm_stats.json", "modeling_molmoact2.py", "configuration_molmoact2.py",
                "processing_molmoact2.py", "tokenizer_config.json", "tokenizer.json"]
    missing.extend(name for name in required if not (checkpoint / name).is_file())
    return {"complete": not missing and size >= total, "missing": missing,
            "weight_files": names, "weight_bytes_present": size, "tensor_bytes_expected": total}


class MolmoAct2Backend:
    def __init__(self, checkpoint, schema, device="cuda:0", dtype="bfloat16", num_steps=10):
        if not inspect_weights(checkpoint)["complete"]:
            raise ValueError("checkpoint download is incomplete; inspect/download before loading the GPU model")
        if not device.startswith("cuda"):
            raise ValueError("this deployment expects a separate CUDA GPU; CPU inference is not started automatically")
        # Keep downloaded remote-code modules in a project-local runtime cache.
        os.environ.setdefault("HF_MODULES_CACHE", str(Path("runs/molmoact2/hf_modules").resolve()))
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable; run this server on the separate GPU PC")
        if dtype == "bfloat16" and not torch.cuda.is_bf16_supported():
            raise RuntimeError("this GPU does not support bfloat16; select --dtype float16 or float32")
        from transformers import AutoModelForImageTextToText, AutoProcessor
        self.torch = torch
        self.schema = schema
        self.num_steps = num_steps
        self.processor = AutoProcessor.from_pretrained(str(checkpoint), trust_remote_code=True,
                                                       local_files_only=True, extra_special_tokens={})
        self.model = AutoModelForImageTextToText.from_pretrained(
            str(checkpoint), trust_remote_code=True, local_files_only=True,
            dtype=getattr(torch, dtype)).to(device).eval()
        self.device = device
        target_dtype = next(self.model.parameters()).dtype

        def move_and_cast(inputs, dev):
            result = {}
            for key, value in inputs.items():
                if torch.is_tensor(value):
                    value = value.to(dev)
                    if value.is_floating_point():
                        value = value.to(target_dtype)
                result[key] = value
            return result

        # Official server uses this instance-level cast for processor float inputs.
        # The pinned checkpoint already fixes trajectory dtype and bf16 -> NumPy.
        self.model._move_inputs_to_device = move_and_cast

    def predict(self, images, state, task):
        with self.torch.inference_mode():
            result = self.model.predict_action(processor=self.processor, images=images, task=task,
                state=state, norm_tag=self.schema["norm_tag"], inference_action_mode="continuous",
                enable_depth_reasoning=False, num_steps=self.num_steps,
                n_action_steps=self.schema["n_action_steps"], normalize_language=True, enable_cuda_graph=False)
        actions = result.actions
        if self.torch.is_tensor(actions):
            actions = actions.detach().to(device="cpu", dtype=self.torch.float32).numpy()
        if actions.ndim == 3 and actions.shape[0] == 1:
            actions = actions[0]
        return check_actions(actions, self.schema)


class InferenceService:
    def __init__(self, schema, backend):
        self.schema, self.backend = schema, backend
        self.lock = threading.Lock()

    def infer(self, payload):
        images, state, task = parse_request(payload, self.schema)
        start = time.monotonic()
        with self.lock:
            actions = check_actions(self.backend.predict(images, state, task), self.schema)
        return {"schema_version": 1, "profile": self.schema["profile"], "request_id": payload["request_id"],
                "actions": actions.tolist(), "action_names": self.schema["action_names"],
                "inference_s": time.monotonic() - start, "aiworker_action_layout": self.schema["aiworker_action_layout"],
                "actuation_enabled": False, "observation_stamps_ns": payload.get("observation_stamps_ns", [])}


def handler_for(service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            # HTTP access lines can contain network identifiers; do not persist them.
            pass

        def reply(self, code, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self.reply(200, {"status": "ok", "model_type": "molmoact2", "actuation_enabled": False})
            elif self.path == "/schema":
                self.reply(200, service.schema)
            else:
                self.reply(404, {"error": "unknown endpoint"})

        def do_POST(self):
            if self.path != "/infer":
                self.reply(404, {"error": "unknown endpoint"})
                return
            try:
                count = int(self.headers.get("Content-Length", "0"))
                if not 0 < count <= MAX_BODY:
                    raise ValueError("request body is empty or exceeds 32 MiB")
                raw = self.rfile.read(count)
                if len(raw) != count:
                    raise ValueError("incomplete request body")
                payload = json.loads(raw)
                parse_request(payload, service.schema)
            except (ValueError, TypeError, KeyError, OSError) as exc:
                self.reply(400, {"error": str(exc)})
                return
            try:
                self.reply(200, service.infer(payload))
            except Exception as exc:
                self.reply(500, {"error": f"inference failed: {type(exc).__name__}: {exc}"})

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--profile", type=Path, default=Path("configs/molmoact2/droid_probe.json"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8210)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--dtype", choices=["bfloat16", "float16", "float32"], default="bfloat16")
    ap.add_argument("--num-steps", type=int, default=10)
    ap.add_argument("--inspect", action="store_true", help="CPU-only metadata/readiness check; does not load Torch")
    ap.add_argument("--warmup", action="store_true")
    args = ap.parse_args(argv)
    if not 1 <= args.num_steps <= 100 or not 1 <= args.port <= 65535:
        ap.error("invalid flow solver steps or port")
    checkpoint = args.checkpoint.resolve()
    schema = checkpoint_schema(checkpoint, Profile.load(args.profile))
    schema["checkpoint_config_sha256"] = hashlib.sha256((checkpoint / "config.json").read_bytes()).hexdigest()
    if args.inspect:
        print(json.dumps({"schema": schema, "weights": inspect_weights(checkpoint)}, indent=2))
        return 0
    backend = MolmoAct2Backend(checkpoint, schema, args.device, args.dtype, args.num_steps)
    if args.warmup:
        images = [np.zeros((256, 256, 3), dtype=np.uint8) for _ in schema["camera_order"]]
        backend.predict(images, np.zeros(schema["state_dim"], np.float32), "warm up")
    server = ThreadingHTTPServer((args.host, args.port), handler_for(InferenceService(schema, backend)))
    server.daemon_threads = True
    print(f"MolmoAct2 ready on {args.host}:{args.port}; profile={schema['profile']}; actuator interface=none", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
