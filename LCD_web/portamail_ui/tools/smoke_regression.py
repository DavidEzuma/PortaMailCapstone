#!/usr/bin/env python3
import os
import sys
import tempfile

from flask import Flask
from flask_socketio import SocketIO

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from interface.api_handlers import register_api
from interface.event_store import read_events
from state.model import snapshot_state
from state.state_machine import ensure_log_dir
from tests.test_support import reset_runtime_state
from transport.socketio_handlers import register_socketio


def _assert(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    reset_runtime_state()

    tmpdir = tempfile.TemporaryDirectory()
    try:
        import state.state_machine as sm

        old_log_dir = sm.LOG_DIR
        old_log_path = sm.LOG_PATH
        sm.LOG_DIR = tmpdir.name
        sm.LOG_PATH = os.path.join(tmpdir.name, "delivery_log.txt")
        ensure_log_dir()

        app = Flask(__name__)
        app.config["SECRET_KEY"] = "smoke-key"
        socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
        emit_state, _emit_events = register_socketio(socketio)
        register_api(app, emit_state)
        client = app.test_client()
        sio_client = socketio.test_client(app)

        resp = client.get("/api/state")
        _assert(resp.status_code == 200, "GET /api/state failed")
        state = resp.get_json()
        _assert(
            set(state.keys()) == {"mode", "screen", "selected_room", "pending_rooms", "active_room", "bits", "events"},
            "state shape changed",
        )

        sio_client.emit("press", {"bit": "room1_start_pressed"})
        sio_client.emit("release", {"bit": "room1_start_pressed", "edge": "start_room1"})
        sio_client.emit("press", {"bit": "room2_start_pressed"})
        sio_client.emit("release", {"bit": "room2_start_pressed", "edge": "start_room2"})
        _assert(len(read_events()) == 2, "dual-room start events missing")

        resp = client.post("/api/mode", json={"mode": "ARRIVED"})
        _assert(resp.status_code == 200, "POST /api/mode ARRIVED failed")

        sio_client.emit("release", {"bit": "confirm_package_pressed", "edge": "open_confirm_flow"})
        sio_client.emit("release", {"bit": "confirm_room1_pressed", "edge": "select_room1"})
        sio_client.emit("release", {"bit": "package_confirmed_pressed", "edge": "delivery_confirmed"})

        with open(sm.LOG_PATH, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
        _assert(len(lines) == 1, "delivery log did not append exactly one line")
        _assert(lines[0].endswith(",ROOM1"), "delivery log room value mismatch")

        st = snapshot_state()
        _assert(st["screen"] == "DELIVERING_ROOM2", "state did not advance to second queued room")
        _assert(st["active_room"] == "ROOM2", "active room did not advance to ROOM2")
        _assert(st["pending_rooms"] == ["ROOM2"], "pending queue did not retain second room")

        resp = client.post("/api/mode", json={"mode": "ARRIVED"})
        _assert(resp.status_code == 200, "second POST /api/mode ARRIVED failed")
        sio_client.emit("release", {"bit": "confirm_package_pressed", "edge": "open_confirm_flow"})
        sio_client.emit("release", {"bit": "confirm_room2_pressed", "edge": "select_room2"})
        sio_client.emit("release", {"bit": "package_confirmed_pressed", "edge": "delivery_confirmed"})

        with open(sm.LOG_PATH, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
        _assert(len(lines) == 2, "second confirmation did not append second log line")
        _assert(lines[1].endswith(",ROOM2"), "second delivery log room value mismatch")

        st = snapshot_state()
        _assert(st["screen"] == "HOME", "state did not return HOME after second delivery")
        _assert(st["mode"] == "DOCK_IDLE", "mode did not return DOCK_IDLE after second delivery")
        _assert(st["pending_rooms"] == [], "pending queue not empty at end")

        sio_client.disconnect()
        print("smoke_regression: ok")
        sm.LOG_DIR = old_log_dir
        sm.LOG_PATH = old_log_path
    finally:
        tmpdir.cleanup()


if __name__ == "__main__":
    main()
