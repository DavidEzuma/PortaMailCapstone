import unittest

from flask import Flask
from flask_socketio import SocketIO

from interface.event_store import read_events
from state.model import snapshot_state
from tests.test_support import TemporaryLogPath, reset_runtime_state
from transport.socketio_handlers import register_socketio


class SocketIoContractTests(unittest.TestCase):
    def setUp(self):
        reset_runtime_state()
        self._log_ctx = TemporaryLogPath()
        self._log_ctx.__enter__()
        self.app = Flask(__name__)
        self.app.config["SECRET_KEY"] = "test-key"
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode="threading")
        register_socketio(self.socketio)
        self.client = self.socketio.test_client(self.app)

    def tearDown(self):
        self.client.disconnect()
        self._log_ctx.__exit__(None, None, None)

    def test_valid_release_generates_single_event(self):
        self.client.emit("press", {"bit": "room1_start_pressed"})
        self.client.emit("release", {"bit": "room1_start_pressed", "edge": "start_room1"})

        events = read_events()
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event["name"], "start_room1")
        self.assertEqual(event["type"], "start_room1")
        self.assertEqual(set(event.keys()), {"ts", "name", "payload", "type", "data"})
        self.assertEqual(event["payload"]["active_room"], "ROOM1")
        self.assertEqual(event["payload"]["pending_rooms"], ["ROOM1"])
        self.assertEqual(event["data"]["screen"], "DELIVERING_ROOM1")
        self.assertEqual(snapshot_state()["bits"]["room1_start_pressed"], 0)

    def test_invalid_edge_for_screen_is_not_recorded(self):
        self.client.emit("release", {"bit": "confirm_package_pressed", "edge": "open_confirm_flow"})
        self.assertEqual(read_events(), [])

    def test_unknown_edge_name_is_not_recorded(self):
        self.client.emit("release", {"bit": "power_pressed", "edge": "not_real"})
        self.assertEqual(read_events(), [])

    def test_power_edge_on_release(self):
        self.client.emit("press", {"bit": "power_pressed"})
        self.client.emit("release", {"bit": "power_pressed", "edge": "power_edge"})
        events = read_events()
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["name"], "power_edge")
        self.assertEqual(snapshot_state()["bits"]["power_pressed"], 0)

    def test_second_room_can_be_queued_after_delivery_started(self):
        self.client.emit("release", {"bit": "room1_start_pressed", "edge": "start_room1"})
        self.client.emit("release", {"bit": "room2_start_pressed", "edge": "start_room2"})
        st = snapshot_state()
        self.assertEqual(st["active_room"], "ROOM1")
        self.assertEqual(st["pending_rooms"], ["ROOM1", "ROOM2"])
        self.assertEqual(st["screen"], "DELIVERING_ROOM1")


if __name__ == "__main__":
    unittest.main()
