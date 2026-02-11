import os
import tempfile

from interface.contract import BIT_KEYS
from interface import event_store
from state import model, state_machine


def reset_runtime_state():
    state = model.get_state()
    state["mode"] = "DOCK_IDLE"
    state["screen"] = "HOME"
    state["selected_room"] = None
    state["pending_rooms"] = []
    state["active_room"] = None
    state["bits"] = {key: 0 for key in BIT_KEYS}
    state["events"] = []
    event_store._events[:] = []


class TemporaryLogPath:
    def __init__(self):
        self._tmpdir = None
        self._old_dir = None
        self._old_path = None
        self.path = None

    def __enter__(self):
        self._tmpdir = tempfile.TemporaryDirectory()
        self._old_dir = state_machine.LOG_DIR
        self._old_path = state_machine.LOG_PATH
        state_machine.LOG_DIR = self._tmpdir.name
        state_machine.LOG_PATH = os.path.join(self._tmpdir.name, "delivery_log.txt")
        state_machine.ensure_log_dir()
        self.path = state_machine.LOG_PATH
        return self.path

    def __exit__(self, exc_type, exc, tb):
        state_machine.LOG_DIR = self._old_dir
        state_machine.LOG_PATH = self._old_path
        self._tmpdir.cleanup()
