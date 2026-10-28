"""Visual-selector contracts with fixture text generation, not real AI inference."""

from dataclasses import dataclass
from http.server import ThreadingHTTPServer
import json
import threading
import unittest

import numpy as np

from wb_eraser.vlm_selector import TargetSelector, parse_selection, parse_selection_request, validate_candidates
from wb_eraser.vlm_server import SelectionService, handler_for


@dataclass
class Region:
    id: int
    x: int = 2
    y: int = 2
    w: int = 5
    h: int = 5


class FixtureVisualBackend:
    def __init__(self, text='{"target_ids":[4]}'):
        self.text, self.calls = text, []

    def generate(self, image, prompt):
        self.calls.append((image.copy(), prompt))
        return self.text


class VisualSelectionTests(unittest.TestCase):
    def test_model_can_select_only_existing_integer_ids(self):
        self.assertEqual(parse_selection('```json\n{"target_ids":[4]}\n```', [4, 9]), [4])
        self.assertEqual(parse_selection('{"target_ids":[]}', [4, 9]), [])
        for text in ['{"target_ids":[99]}', '{"target_ids":[true]}', '{"target_ids":[4,4]}',
                     '{"target_ids":["4"]}', 'erase everything', '{"poses":[[0,0,0]]}']:
            with self.assertRaises(ValueError):
                parse_selection(text, [4, 9])

    def test_candidate_boxes_cannot_extend_outside_observation(self):
        validate_candidates([{"id": 0, "bbox": [1, 1, 5, 5]}], (8, 12, 3))
        for box in [[-1, 0, 5, 5], [10, 1, 5, 5], [0, 0, 0, 2]]:
            with self.assertRaises(ValueError):
                validate_candidates([{"id": 0, "bbox": box}], (8, 12, 3))

    def test_image_and_response_identity_are_bound_to_request(self):
        backend = FixtureVisualBackend()
        service = SelectionService(backend, {"backend": "fixture_not_ai", "is_ai_model": False})
        def exchange(method, path, raw):
            if path == "/health":
                return service.health()
            request = json.loads(raw)
            image, ids, prompt = parse_selection_request(request)
            np.testing.assert_array_equal(image, np.full((8, 12, 3), 111, np.uint8))
            self.assertEqual(ids, [4, 9])
            self.assertIn("erase only selected writing", prompt)
            return service.select(request)
        client = TargetSelector(exchange=exchange)
        ids, record = client.select(np.full((8, 12, 3), 111, np.uint8), [Region(4), Region(9)],
                                    "erase only selected writing")
        self.assertEqual(ids, [4])
        self.assertFalse(record["service"]["is_ai_model"])
        self.assertEqual(len(backend.calls), 1)

    def test_invalid_model_text_is_recorded_as_failed_selection(self):
        service = SelectionService(FixtureVisualBackend("cannot determine the regions"))
        def exchange(method, path, raw):
            return service.health() if path == "/health" else service.select(json.loads(raw))
        with self.assertRaisesRegex(ValueError, "raw_text=cannot determine"):
            TargetSelector(exchange=exchange).select(np.zeros((8, 12, 3), np.uint8), [Region(4)], "erase")

    def test_swapped_response_rejected(self):
        service = SelectionService(FixtureVisualBackend())
        def exchange(method, path, raw):
            if path == "/health":
                return service.health()
            result = service.select(json.loads(raw))
            result["image_sha256"] = "different observation"
            return result
        with self.assertRaisesRegex(ValueError, "does not match"):
            TargetSelector(exchange=exchange).select(np.zeros((8, 12, 3), np.uint8), [Region(4)], "erase")

    def test_http_roundtrip_uses_real_png_and_fixture_generation(self):
        backend = FixtureVisualBackend()
        service = SelectionService(backend, {"backend": "fixture_not_ai", "is_ai_model": False})
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(service))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = TargetSelector(f"http://127.0.0.1:{server.server_port}")
            selected, _ = client.select(np.zeros((8, 12, 3), np.uint8), [Region(4)], "erase")
            self.assertEqual(selected, [4])
            self.assertEqual(len(backend.calls), 1)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)


if __name__ == "__main__":
    unittest.main()
