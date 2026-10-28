"""Lightweight HTTP client and file-based MolmoAct2 inference. No Torch or ROS."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

import numpy as np

from .molmoact2_contract import MAX_BODY, Profile, check_actions, make_request, validate_schema


class MolmoAct2Client:
    def __init__(self, profile, url="http://127.0.0.1:8210", timeout=30., exchange=None):
        parsed = urllib.parse.urlsplit(url)
        if (parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.username or parsed.password
                or parsed.path not in ("", "/") or parsed.query or parsed.fragment or not 0 < timeout <= 300):
            raise ValueError("expected a plain HTTP(S) server URL and a positive bounded timeout")
        self.profile, self.url, self.timeout = profile, url.rstrip("/"), timeout
        self.exchange = exchange
        self.schema = None

    def request(self, method, path, payload=None):
        raw = json.dumps(payload, allow_nan=False).encode() if payload is not None else None
        if raw is not None and len(raw) > MAX_BODY:
            raise ValueError("encoded observation exceeds the request size limit")
        if self.exchange is not None:
            return self.exchange(method, path, raw)
        req = urllib.request.Request(self.url + path, data=raw, method=method,
                                     headers={"Content-Type": "application/json", "Accept": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as response:
                body = response.read(MAX_BODY + 1)
                if len(body) > MAX_BODY:
                    raise ValueError("server response exceeds the size limit")
                return json.loads(body)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f"MolmoAct2 HTTP {exc.code}: " + exc.read(8192).decode(errors="replace")) from exc
        except (socket.timeout, TimeoutError) as exc:
            raise TimeoutError("MolmoAct2 inference server timed out") from exc
        except urllib.error.URLError as exc:
            raise ConnectionError("could not reach the MolmoAct2 server; check the GPU server and SSH forward") from exc

    def check_server(self):
        health = self.request("GET", "/health")
        if health.get("status") != "ok" or health.get("actuation_enabled") is not False:
            raise ValueError("unexpected server health or actuator state")
        self.schema = validate_schema(self.request("GET", "/schema"), self.profile)
        return self.schema

    def infer(self, images, names, positions, instruction, stamps_ns=None):
        if self.schema is None:
            self.check_server()
        request_id = uuid.uuid4().hex
        payload = make_request(images, names, positions, instruction, self.schema, request_id, stamps_ns)
        started = time.monotonic()
        result = self.request("POST", "/infer", payload)
        if (result.get("schema_version") != 1 or result.get("request_id") != request_id
                or result.get("profile") != self.schema["profile"] or result.get("actuation_enabled") is not False
                or result.get("action_names") != self.schema["action_names"]
                or result.get("aiworker_action_layout") != self.schema["aiworker_action_layout"]
                or result.get("observation_stamps_ns") != payload["observation_stamps_ns"]):
            raise ValueError("response identity, timestamps, joint labels or actuator state mismatch")
        checked = check_actions(result["actions"], self.schema)
        return checked, {"request_id": request_id, "roundtrip_s": time.monotonic() - started,
                         "inference_s": result["inference_s"], "action_names": self.schema["action_names"],
                         "aiworker_action_layout": self.schema["aiworker_action_layout"], "actuator_commands_sent": 0}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("command", choices=["probe", "infer-file"])
    ap.add_argument("--profile", type=Path, default=Path("configs/molmoact2/droid_probe.json"))
    ap.add_argument("--url", default="http://127.0.0.1:8210")
    ap.add_argument("--timeout", type=float, default=30.)
    ap.add_argument("--input", type=Path, help="private NPZ with camera arrays, joint_names and positions (no pickled objects)")
    ap.add_argument("--instruction")
    ap.add_argument("--out", type=Path, default=Path("runs/molmoact2/client_result.json"))
    args = ap.parse_args(argv)
    if args.command == "infer-file" and (args.input is None or not args.instruction):
        ap.error("infer-file requires --input and --instruction")
    client = MolmoAct2Client(Profile.load(args.profile), args.url, args.timeout)
    report = {"complete": False, "actuator_commands_sent": 0, "mode": args.command}
    try:
        report["server"] = client.check_server()
        if args.command == "infer-file":
            with np.load(args.input, allow_pickle=False) as observation:
                images = {name: observation[name] for name in set(client.schema["camera_order"])}
                actions, info = client.infer(images, observation["joint_names"].tolist(), observation["positions"], args.instruction)
            report.update(info, actions=actions.tolist(), action_shape=list(actions.shape), offline_input=True,
                          live_observation_freshness_verified=False)
        report["complete"] = True
    except Exception as exc:
        report["error"] = str(exc)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"MolmoAct2 {args.command}: complete={report['complete']}; report={args.out}; actuator commands=0")
    if "error" in report:
        print(report["error"])
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
