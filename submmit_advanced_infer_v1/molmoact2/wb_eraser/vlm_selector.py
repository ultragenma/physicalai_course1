"""Bounded visual selection of detected ink regions; no arbitrary robot poses."""

from __future__ import annotations

import hashlib
import json
import time
import uuid

import numpy as np

from .molmoact2_client import MolmoAct2Client
from .molmoact2_contract import decode_image, encode_image


def image_hash(rgb):
    array = np.asarray(rgb)
    return hashlib.sha256(str(array.shape).encode() + array.tobytes()).hexdigest()


def candidate_list(components):
    return [{"id": int(c.id), "bbox": [int(c.x), int(c.y), int(c.w), int(c.h)]} for c in components]


def validate_candidates(candidates, shape):
    if not isinstance(candidates, list) or not 1 <= len(candidates) <= 128:
        raise ValueError("one to 128 detected writing regions are required")
    ids = []
    h, w = shape[:2]
    for item in candidates:
        if not isinstance(item, dict) or type(item.get("id")) is not int or not 0 <= item["id"] < 10000:
            raise ValueError("invalid candidate ID")
        box = item.get("bbox")
        if not isinstance(box, list) or len(box) != 4 or any(type(v) is not int for v in box):
            raise ValueError("candidate bounds must be four integers")
        x, y, width, height = box
        if x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > w or y + height > h:
            raise ValueError("candidate box lies outside the observed image")
        ids.append(item["id"])
    if len(ids) != len(set(ids)):
        raise ValueError("candidate IDs must be unique")
    return ids


def parse_selection(text, allowed_ids):
    """Accept JSON (optionally fenced), but never execute model-written code."""
    if not isinstance(text, str) or len(text) > 16000:
        raise ValueError("invalid model text")
    start = text.find("{")
    if start < 0:
        raise ValueError("model did not return a JSON object")
    value, _ = json.JSONDecoder().raw_decode(text[start:])
    ids = value.get("target_ids") if isinstance(value, dict) else None
    if not isinstance(ids, list) or any(type(i) is not int for i in ids):
        raise ValueError("target_ids must be a list of integer candidate IDs")
    if len(set(ids)) != len(ids) or not set(ids) <= set(allowed_ids):
        raise ValueError("model selected duplicated or unknown writing regions")
    return ids


def selection_prompt(task, candidates):
    return ("The image shows a whiteboard. Green numbered boxes identify candidate writing regions. "
            "Select the regions to erase according to the instruction. Select only IDs from the candidate list. "
            "Do not select the board frame or the printed ID labels themselves. "
            "For erase-all instructions select every candidate writing region. "
            "Return only JSON of the form {\"target_ids\": [0, 1]}; use [] if no region should be erased.\n"
            "Instruction: " + task + "\nCandidates: " + json.dumps(candidates))


def parse_selection_request(payload):
    if (not isinstance(payload, dict) or payload.get("schema_version") != 1
            or not isinstance(payload.get("request_id"), str) or not 1 <= len(payload["request_id"]) <= 128
            or not isinstance(payload.get("instruction"), str) or not payload["instruction"].strip()
            or len(payload["instruction"]) > 2000):
        raise ValueError("invalid visual selection request")
    rgb = decode_image(payload["image"])
    if image_hash(rgb) != payload.get("image_sha256"):
        raise ValueError("selection image hash mismatch")
    ids = validate_candidates(payload["candidates"], rgb.shape)
    return rgb, ids, selection_prompt(payload["instruction"], payload["candidates"])


class TargetSelector:
    def __init__(self, url="http://127.0.0.1:8211", timeout=45., exchange=None):
        # Share the bounded JSON/HTTP transport; no action profile is involved.
        self.transport = MolmoAct2Client(None, url, timeout, exchange)
        self.info = None

    def check_server(self):
        self.info = self.transport.request("GET", "/health")
        if (self.info.get("status") != "ok" or self.info.get("service") != "visual_target_selection"
                or self.info.get("actuation_enabled") is not False):
            raise ValueError("unexpected visual selector service")
        return self.info

    def select(self, rgb, components, instruction):
        if self.info is None:
            self.check_server()
        candidates = candidate_list(components)
        allowed = validate_candidates(candidates, rgb.shape)
        request = {"schema_version": 1, "request_id": uuid.uuid4().hex, "instruction": instruction,
                   "image": encode_image(rgb), "image_sha256": image_hash(rgb), "candidates": candidates}
        start = time.monotonic()
        response = self.transport.request("POST", "/select", request)
        if (response.get("schema_version") != 1 or response.get("request_id") != request["request_id"]
                or response.get("image_sha256") != request["image_sha256"]
                or response.get("actuation_enabled") is not False):
            raise ValueError("visual selection response does not match its source image/request")
        if response.get("selection_valid") is not True:
            raise ValueError("model selection invalid: " + str(response.get("error"))
                             + "; raw_text=" + str(response.get("raw_text", ""))[:2000])
        selected = parse_selection(json.dumps({"target_ids": response.get("target_ids")}), allowed)
        return selected, {"source": "vlm", "service": self.info, "request_id": request["request_id"],
                          "image_sha256": request["image_sha256"], "target_ids": selected,
                          "raw_text": response.get("raw_text", ""), "roundtrip_s": time.monotonic() - start}
