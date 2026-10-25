#!/usr/bin/env python3
"""Lightweight ZMQ client for the NVIDIA Isaac-GR00T PolicyServer API.

This adapter follows the MessagePack/NumPy wire protocol in the pinned GR00T
server revision. It avoids importing ``gr00t.policy`` on the ROS client, whose
package initializer also imports model-side torch/transformers dependencies.
"""
from __future__ import annotations

import functools
import io
import json
from typing import Any

import msgpack
import msgpack_numpy as mnp
import numpy as np
import zmq


class _Serializer:
    @staticmethod
    def _encode_custom(value: Any) -> Any:
        return value

    @staticmethod
    def _decode_custom(value: Any) -> Any:
        if not isinstance(value, dict):
            return value
        marker = value.get("__ModalityConfig__", value.get(b"__ModalityConfig__"))
        if marker is None:
            marker = value.get("__ModalityConfig_class__", value.get(b"__ModalityConfig_class__"))
        if not marker:
            return value
        payload = value.get("as_json", value.get(b"as_json"))
        if payload is None:
            raise ValueError("malformed GR00T modality-config response")
        if isinstance(payload, bytes):
            payload = payload.decode("utf-8")
        return json.loads(payload) if isinstance(payload, str) else payload

    @staticmethod
    def _safe_encode(value: Any, chain=None) -> Any:
        if isinstance(value, np.ndarray) and value.dtype.kind == "O":
            raise TypeError("object-dtype NumPy arrays are not allowed in GR00T observations")
        return mnp.encode(value, chain=chain)

    @staticmethod
    def _safe_decode(value: Any, chain=None) -> Any:
        if isinstance(value, dict):
            marker = value.get("__ndarray_class__", value.get(b"__ndarray_class__"))
            if marker:
                payload = value.get("as_npy", value.get(b"as_npy"))
                if payload is None:
                    raise ValueError("malformed GR00T ndarray response")
                return np.load(io.BytesIO(payload), allow_pickle=False)
            nd_value = value.get(b"nd", value.get("nd"))
            kind_value = value.get(b"kind", value.get("kind"))
            if nd_value and kind_value in (b"O", "O"):
                raise ValueError("object-dtype NumPy payloads are not allowed")
        return mnp.decode(value, chain=chain)

    @classmethod
    def to_bytes(cls, value: Any) -> bytes:
        default = functools.partial(cls._safe_encode, chain=cls._encode_custom)
        return msgpack.packb(value, default=default)

    @classmethod
    def from_bytes(cls, value: bytes) -> Any:
        object_hook = functools.partial(cls._safe_decode, chain=cls._decode_custom)
        return msgpack.unpackb(value, object_hook=object_hook, raw=False)


class GrootPolicyClient:
    """Subset of NVIDIA PolicyClient used by the ROS 2 bridge."""

    def __init__(self, host: str, port: int, timeout_ms: int) -> None:
        if not host or not 1 <= port <= 65535 or timeout_ms <= 0:
            raise ValueError("invalid GR00T PolicyServer host, port, or timeout")
        if host != "127.0.0.1":
            raise ValueError("GR00T PolicyServer must use the documented 127.0.0.1 SSH local-forward")
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self.context = zmq.Context()
        self.socket: zmq.Socket | None = None
        self._closed = False
        self._init_socket()

    def _init_socket(self) -> None:
        if self.socket is not None:
            self.socket.close(linger=0)
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(f"tcp://{self.host}:{self.port}")

    def _call(self, endpoint: str, data: dict[str, Any] | None = None) -> Any:
        if self._closed or self.socket is None:
            raise RuntimeError("GR00T PolicyClient is closed")
        request: dict[str, Any] = {"endpoint": endpoint}
        if data is not None:
            request["data"] = data
        try:
            self.socket.send(_Serializer.to_bytes(request))
            response_bytes = self.socket.recv()
        except zmq.error.Again:
            self._init_socket()
            raise TimeoutError(f"GR00T PolicyServer request timed out ({endpoint})") from None
        except zmq.error.ZMQError:
            self._init_socket()
            raise
        response = _Serializer.from_bytes(response_bytes)
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"GR00T PolicyServer error: {response['error']}")
        return response

    def ping(self) -> bool:
        try:
            self._call("ping")
            return True
        except (TimeoutError, zmq.error.ZMQError, RuntimeError):
            return False

    def get_modality_config(self) -> dict[str, Any]:
        result = self._call("get_modality_config")
        if not isinstance(result, dict):
            raise TypeError("PolicyServer get_modality_config response must be a mapping")
        return result

    def get_action(self, observation: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        result = self._call("get_action", {"observation": observation, "options": None})
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise TypeError("PolicyServer get_action response must contain (action, info)")
        action, info = result
        if not isinstance(action, dict) or not isinstance(info, dict):
            raise TypeError("PolicyServer action and info responses must be mappings")
        return action, info

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        self.context.term()
