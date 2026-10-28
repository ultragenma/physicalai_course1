"""MolmoAct2 vision/language target selector, independent of robot action dimensions."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time

from PIL import Image

from .molmoact2_contract import MAX_BODY
from .molmoact2_server import MolmoModel, inspect_weights
from .vlm_selector import parse_selection, parse_selection_request


class MolmoVisualBackend(MolmoModel):
    def __init__(self, checkpoint, device="cuda:0", dtype="bfloat16", max_new_tokens=512):
        super().__init__(checkpoint, device, dtype)
        self.max_new_tokens = max_new_tokens

    def generate(self, rgb, prompt):
        messages = [{"role": "user", "content": [
            {"type": "image", "image": Image.fromarray(rgb)}, {"type": "text", "text": prompt}]}]
        inputs = self.processor.apply_chat_template(messages, tokenize=True, add_generation_prompt=True,
                                                     return_tensors="pt", return_dict=True)
        inputs = self.model._move_inputs_to_device(inputs, self.device)
        dtype = next(self.model.parameters()).dtype
        with self.torch.inference_mode(), self.torch.autocast("cuda", dtype=dtype, enabled=dtype != self.torch.float32):
            generated = self.model.generate(**inputs, max_new_tokens=self.max_new_tokens, do_sample=False,
                                             return_dict_in_generate=False, use_cache=True)
        generated = generated[:, inputs["input_ids"].shape[1]:].detach().cpu()
        return self.processor.post_process_image_text_to_text(
            generated, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0]


class SelectionService:
    def __init__(self, backend, model_info=None):
        self.backend = backend
        self.model_info = model_info or {"backend": "molmoact2_vlm", "is_ai_model": True}
        self.lock = threading.Lock()

    def health(self):
        return {"status": "ok", "service": "visual_target_selection", "schema_version": 1,
                "actuation_enabled": False, **self.model_info}

    def select(self, payload):
        rgb, allowed, prompt = parse_selection_request(payload)
        start = time.monotonic()
        with self.lock:
            raw_text = self.backend.generate(rgb, prompt)
        result = {"schema_version": 1, "request_id": payload["request_id"],
                  "image_sha256": payload["image_sha256"], "raw_text": raw_text,
                  "generation_s": time.monotonic() - start, "actuation_enabled": False}
        try:
            result.update(target_ids=parse_selection(raw_text, allowed), selection_valid=True)
        except (ValueError, TypeError) as exc:
            result.update(target_ids=None, selection_valid=False, error=str(exc))
        return result


def handler_for(service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def reply(self, code, value):
            data = json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/health":
                self.reply(200, service.health())
            else:
                self.reply(404, {"error": "unknown endpoint"})

        def do_POST(self):
            if self.path != "/select":
                self.reply(404, {"error": "unknown endpoint"})
                return
            try:
                count = int(self.headers.get("Content-Length", "0"))
                if not 0 < count <= MAX_BODY:
                    raise ValueError("invalid or oversized request")
                payload = json.loads(self.rfile.read(count))
                parse_selection_request(payload)
            except (ValueError, TypeError, KeyError, OSError) as exc:
                self.reply(400, {"error": str(exc)})
                return
            try:
                self.reply(200, service.select(payload))
            except Exception as exc:
                self.reply(500, {"error": f"model generation failed: {type(exc).__name__}: {exc}"})

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--checkpoint", type=Path, default=Path("checkpoints/molmoact2-droid"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8211)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--dtype", choices=["bfloat16", "float16", "float32"], default="bfloat16")
    ap.add_argument("--max-new-tokens", type=int, default=512)
    ap.add_argument("--inspect", action="store_true")
    args = ap.parse_args()
    if not 32 <= args.max_new_tokens <= 1024 or not 1 <= args.port <= 65535:
        ap.error("invalid generation length or port")
    checkpoint = args.checkpoint.resolve()
    if args.inspect:
        print(json.dumps({"service": "visual_target_selection", "weights": inspect_weights(checkpoint),
                          "mode": "MolmoAct2 text generation from image; no joint actions or FT", "gpu_forward_executed": False}, indent=2))
        return
    backend = MolmoVisualBackend(checkpoint, args.device, args.dtype, args.max_new_tokens)
    server = ThreadingHTTPServer((args.host, args.port), handler_for(SelectionService(backend)))
    server.daemon_threads = True
    print(f"MolmoAct2 visual selector ready at {args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
