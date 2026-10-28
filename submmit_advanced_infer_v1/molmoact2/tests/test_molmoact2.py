"""CPU protocol/contract tests with a fixture backend, not model inference."""

import copy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

import numpy as np

from wb_eraser.robot_schema import CAMERAS, STATIONARY_JOINTS
from wb_eraser.molmoact2_client import MolmoAct2Client
from wb_eraser.molmoact2_contract import (Profile, check_actions, checkpoint_schema, decode_image, encode_image,
                                         make_request, ordered_state, parse_request, validate_schema)
from wb_eraser.molmoact2_server import InferenceService, handler_for, inspect_weights

ROOT = Path(__file__).resolve().parents[1]


def profile(kind="droid_probe"):
    return Profile.load(ROOT / f"configs/molmoact2/{kind}.json")


def checkpoint_fixture(path, selected=None):
    selected = selected or profile()
    dimension = len(selected.joint_order)
    stats = {"min": [-1.] * dimension, "max": [1.] * dimension}
    metadata = {"state_stats": stats, "action_stats": copy.deepcopy(stats), "action_horizon": 15,
                "n_action_steps": 15}
    if selected.kind == "sg2_native":
        metadata.update(robot_type="robotis_aiworker_sg2", state_joint_names=list(selected.joint_order),
                        action_joint_names=list(selected.action_names), camera_order=list(selected.camera_order),
                        action_representation="absolute_joint_positions")
    config = {"model_type": "molmoact2", "add_action_expert": True, "max_action_horizon": 30, "max_action_dim": 32}
    (path / "config.json").write_text(json.dumps(config))
    (path / "norm_stats.json").write_text(json.dumps({"norm_mode": "min_max", "metadata_by_tag": {selected.norm_tag: metadata}}))
    return checkpoint_schema(path, selected)


class FixtureBackend:
    """Copies state into a fake chunk; explicitly not an AI policy."""
    def __init__(self, schema):
        self.schema = schema
        self.calls = []

    def predict(self, images, state, task):
        self.calls.append((images, state.copy(), task))
        return np.repeat(state[None, :], self.schema["n_action_steps"], axis=0)


class MolmoFixture:
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        self.profile = profile()
        self.schema = checkpoint_fixture(self.path)
        self.images = {name: np.full((8, 12, 3), i * 50, np.uint8) for i, name in enumerate(CAMERAS)}

    def tearDown(self):
        self.tmp.cleanup()

    def packet(self):
        return make_request(self.images, list(reversed(STATIONARY_JOINTS)), list(reversed(range(19))),
                            "Erase the marks.", self.schema, "fixture_request", [1, 2, 3, 4])


class MolmoContractTests(MolmoFixture, unittest.TestCase):
    def test_named_state_and_three_camera_slots_roundtrip(self):
        images, state, task = parse_request(self.packet(), self.schema)
        np.testing.assert_array_equal(state, np.arange(8, 16))
        self.assertEqual(len(images), 3)
        np.testing.assert_array_equal(images[0], self.images[CAMERAS[0]])
        np.testing.assert_array_equal(images[0], images[1])
        np.testing.assert_array_equal(images[2], self.images[CAMERAS[2]])
        self.assertEqual(task, "Erase the marks.")

    def test_missing_duplicate_and_nonfinite_joint_states_rejected(self):
        for names, values in [(["same"] * 8, [0] * 8), ([], []),
                              (list(self.profile.joint_order), [float("nan")] * 8)]:
            with self.assertRaises(ValueError):
                ordered_state(names, values, self.profile.joint_order)

    def test_images_are_lossless_and_require_rgb_uint8(self):
        image = np.arange(288, dtype=np.uint8).reshape(8, 12, 3)
        np.testing.assert_array_equal(decode_image(encode_image(image)), image)
        for bad in [image.astype(np.float32), image[:, :, 0], image[:0]]:
            with self.assertRaises(ValueError):
                encode_image(bad)

    def test_droid_checkpoint_is_never_claimed_aiworker_compatible(self):
        self.assertFalse(self.schema["aiworker_action_layout"])
        self.assertFalse(self.schema["state_semantics_verified"])
        self.assertFalse(self.schema["actuation_enabled"])
        self.assertTrue(all(name.startswith("franka_") for name in self.schema["action_names"]))
        with self.assertRaisesRegex(ValueError, "normalization tag"):
            checkpoint_schema(self.path, profile("sg2_native"))

    def test_native_policy_requires_identity_and_named_order_not_just_19d(self):
        selected = profile("sg2_native")
        schema = checkpoint_fixture(self.path, selected)
        self.assertEqual(schema["action_dim"], 19)
        self.assertTrue(schema["aiworker_action_layout"])
        payload = json.loads((self.path / "norm_stats.json").read_text())
        payload["metadata_by_tag"][selected.norm_tag]["action_joint_names"] = list(reversed(selected.action_names))
        (self.path / "norm_stats.json").write_text(json.dumps(payload))
        with self.assertRaisesRegex(ValueError, "named state/action"):
            checkpoint_schema(self.path, selected)

    def test_tag_horizon_is_read_from_checkpoint_not_assumed_16(self):
        self.assertEqual(self.schema["n_action_steps"], 15)
        payload = json.loads((self.path / "norm_stats.json").read_text())
        payload["metadata_by_tag"][self.profile.norm_tag]["action_horizon"] = 31
        (self.path / "norm_stats.json").write_text(json.dumps(payload))
        with self.assertRaises(ValueError):
            checkpoint_schema(self.path, self.profile)

    def test_whole_chunk_checked_including_last_action(self):
        actions = np.zeros((15, 8), np.float32)
        self.assertEqual(check_actions(actions, self.schema).shape, (15, 8))
        actions[-1, -1] = np.inf
        with self.assertRaises(ValueError):
            check_actions(actions, self.schema)
        with self.assertRaises(ValueError):
            check_actions(np.zeros((16, 19)), self.schema)

    def test_packet_profile_camera_and_order_mismatch_rejected(self):
        for change in [lambda p: p.update(profile="other"),
                       lambda p: p.update(joint_order=list(reversed(p["joint_order"]))),
                       lambda p: p["images"].pop(CAMERAS[2])]:
            packet = self.packet()
            change(packet)
            with self.assertRaises(ValueError):
                parse_request(packet, self.schema)

    def test_missing_weights_reported_without_loading_torch(self):
        result = inspect_weights(self.path)
        self.assertFalse(result["complete"])
        self.assertIn("model.safetensors", result["missing"])

    def test_schema_mismatches_fail_before_sending_images(self):
        for key, value in [("actuation_enabled", True), ("joint_order", []), ("camera_order", []),
                           ("action_dim", 19), ("n_action_steps", 31)]:
            bad = copy.deepcopy(self.schema)
            bad[key] = value
            with self.assertRaises(ValueError):
                validate_schema(bad, self.profile)


class MolmoClientTests(MolmoFixture, unittest.TestCase):
    def setUp(self):
        super().setUp()
        self.backend = FixtureBackend(self.schema)
        self.service = InferenceService(self.schema, self.backend)

    def exchange(self, method, endpoint, raw):
        if endpoint == "/health":
            return {"status": "ok", "actuation_enabled": False}
        if endpoint == "/schema":
            return self.schema
        return self.service.infer(json.loads(raw))

    def test_client_service_exchange_and_named_predictions(self):
        client = MolmoAct2Client(self.profile, exchange=self.exchange)
        actions, info = client.infer(self.images, list(STATIONARY_JOINTS), list(range(19)), "Erase marks")
        np.testing.assert_array_equal(actions, np.tile(np.arange(8, 16), (15, 1)))
        self.assertEqual(info["actuator_commands_sent"], 0)
        self.assertFalse(info["aiworker_action_layout"])
        self.assertEqual(len(self.backend.calls), 1)

    def test_corrupted_response_identity_or_chunk_is_rejected(self):
        for change in [lambda r: r.update(request_id="wrong"), lambda r: r.update(action_names=["arm_r_joint1"] * 8),
                       lambda r: r["actions"][-1].__setitem__(-1, float("nan")),
                       lambda r: r.update(observation_stamps_ns=[999]), lambda r: r.update(actuation_enabled=True)]:
            def exchange(method, endpoint, raw):
                reply = self.exchange(method, endpoint, raw)
                if endpoint == "/infer":
                    change(reply)
                return reply
            client = MolmoAct2Client(self.profile, exchange=exchange)
            with self.assertRaises(ValueError):
                client.infer(self.images, list(STATIONARY_JOINTS), list(range(19)), "Erase")

    def test_only_plain_urls_and_bounded_timeouts_are_accepted(self):
        for url, timeout in [("file:///tmp/model", 1), ("http://user:secret@localhost:8210", 1),
                              ("http://localhost:8210/infer", 1), ("http://localhost:8210", float("nan"))]:
            with self.assertRaises(ValueError):
                MolmoAct2Client(self.profile, url, timeout)


class HTTPIntegrationTests(unittest.TestCase):
    def test_real_http_request_decodes_multiview_state_and_response(self):
        with tempfile.TemporaryDirectory() as tmp:
            schema = checkpoint_fixture(Path(tmp))
            backend = FixtureBackend(schema)
            server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(InferenceService(schema, backend)))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            url = f"http://127.0.0.1:{server.server_port}"
            try:
                client = MolmoAct2Client(profile(), url)
                images = {name: np.zeros((8, 12, 3), np.uint8) for name in CAMERAS}
                actions, _ = client.infer(images, list(STATIONARY_JOINTS), list(range(19)), "fixture, not an AI policy")
                self.assertEqual(actions.shape, (15, 8))
                self.assertEqual(len(backend.calls), 1)
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(urllib.request.Request(url + "/infer", data=b"not-json"))
                self.assertEqual(error.exception.code, 400)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=3)


if __name__ == "__main__":
    unittest.main()
