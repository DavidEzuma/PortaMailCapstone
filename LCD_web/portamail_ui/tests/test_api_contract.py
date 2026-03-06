import time
import unittest

from flask import Flask

from interface.api_handlers import register_api
from interface.event_store import add_event, read_events
from state.model import snapshot_state
from tests.test_support import TemporaryLogPath, reset_runtime_state


class ApiContractTests(unittest.TestCase):
    def setUp(self):
        reset_runtime_state()
        self._log_ctx = TemporaryLogPath()
        self._log_ctx.__enter__()
        app = Flask(__name__)
        register_api(app, emit_state=lambda: None)
        self.client = app.test_client()

    def tearDown(self):
        self._log_ctx.__exit__(None, None, None)

    def test_get_state_shape_is_stable(self):
        resp = self.client.get("/api/state")
        self.assertEqual(resp.status_code, 200)
        data = resp.get_json()
        self.assertIsInstance(data, dict)
        self.assertEqual(
            set(data.keys()),
            {"mode", "screen", "selected_room", "pending_rooms", "active_room", "bits", "events"},
        )
        self.assertEqual(data["mode"], "DOCK_IDLE")
        self.assertEqual(data["screen"], "HOME")

    def test_get_events_returns_list(self):
        resp = self.client.get("/api/events")
        self.assertEqual(resp.status_code, 200)
        data = resp.get_json()
        self.assertIsInstance(data, list)
        self.assertEqual(data, [])
        self.assertEqual(read_events(), [])

    def test_get_events_supports_limit_query(self):
        add_event("start_room1")
        time.sleep(0.002)
        second = add_event("start_room2")
        resp = self.client.get("/api/events?limit=1")
        self.assertEqual(resp.status_code, 200)
        data = resp.get_json()
        self.assertEqual(len(data), 1)
        self.assertEqual(data[0]["name"], second["name"])
        self.assertEqual(data[0]["ts"], second["ts"])

    def test_get_events_supports_since_ts_query(self):
        first = add_event("start_room1")
        time.sleep(0.002)
        second = add_event("start_room2")
        resp = self.client.get("/api/events", query_string={"since_ts": first["ts"]})
        self.assertEqual(resp.status_code, 200)
        data = resp.get_json()
        self.assertEqual(len(data), 1)
        self.assertEqual(data[0]["name"], second["name"])
        self.assertEqual(data[0]["ts"], second["ts"])

    def test_get_events_ignores_invalid_limit(self):
        add_event("start_room1")
        add_event("start_room2")
        resp = self.client.get("/api/events?limit=0")
        self.assertEqual(resp.status_code, 200)
        data = resp.get_json()
        self.assertEqual(len(data), 2)

    def test_post_mode_accepts_arrived(self):
        resp = self.client.post("/api/mode", json={"mode": "ARRIVED"})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.get_json(), {"ok": True, "mode": "ARRIVED"})
        state = snapshot_state()
        self.assertEqual(state["mode"], "ARRIVED")
        self.assertEqual(state["screen"], "ARRIVED")

    def test_post_mode_rejects_unknown_mode(self):
        resp = self.client.post("/api/mode", json={"mode": "DELIVERING_ROOM1"})
        self.assertEqual(resp.status_code, 400)
        self.assertEqual(resp.get_json(), {"ok": False, "error": "unknown_mode"})


if __name__ == "__main__":
    unittest.main()
