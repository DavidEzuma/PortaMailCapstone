import os
from datetime import datetime
from interface.validators import validate_mode, validate_edge_for_screen
from state.model import get_state

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(os.path.dirname(BASE_DIR), "logs")
LOG_PATH = os.path.join(LOG_DIR, "delivery_log.txt")


def ensure_log_dir():
    os.makedirs(LOG_DIR, exist_ok=True)
    if not os.path.exists(LOG_PATH):
        with open(LOG_PATH, "a", encoding="utf-8") as _:
            pass


def append_log_line(room):
    ensure_log_dir()
    timestamp = datetime.now().astimezone().isoformat()
    line = f"{timestamp},{room}"
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def set_mode(mode):
    if not validate_mode(mode):
        return False
    state = get_state()
    state["mode"] = mode
    if mode == "ARRIVED":
        state["screen"] = "ARRIVED"
    else:
        state["screen"] = "HOME"
        state["selected_room"] = None
    return True


def reset_to_home():
    state = get_state()
    state["selected_room"] = None
    state["active_room"] = None
    state["pending_rooms"] = []
    state["mode"] = "DOCK_IDLE"
    state["screen"] = "HOME"


def enqueue_room(room):
    state = get_state()
    if room not in {"ROOM1", "ROOM2"}:
        return False
    if room not in state["pending_rooms"]:
        state["pending_rooms"].append(room)
    # Enforce priority: ROOM1 before ROOM2
    state["pending_rooms"].sort(key=lambda r: 0 if r == "ROOM1" else 1)
    return True


def _set_active_from_queue():
    state = get_state()
    if not state["pending_rooms"]:
        return False
    next_room = state["pending_rooms"][0]
    state["active_room"] = next_room
    state["screen"] = "DELIVERING_ROOM1" if next_room == "ROOM1" else "DELIVERING_ROOM2"
    return True


def handle_edge(edge, payload=None):
    state = get_state()
    if not validate_edge_for_screen(edge, state["screen"], state["mode"], state["selected_room"]):
        return False

    if edge == "start_room1":
        enqueue_room("ROOM1")
        if state["screen"] not in {"DELIVERING_ROOM1", "DELIVERING_ROOM2", "CONFIRM_SELECT", "CONFIRM_ACK"}:
            _set_active_from_queue()
        return True
    if edge == "start_room2":
        enqueue_room("ROOM2")
        if state["screen"] not in {"DELIVERING_ROOM1", "DELIVERING_ROOM2", "CONFIRM_SELECT", "CONFIRM_ACK"}:
            _set_active_from_queue()
        return True
    if edge == "power_edge":
        return True
    if edge == "open_confirm_flow":
        state["screen"] = "CONFIRM_SELECT"
        return True
    if edge == "select_room1":
        state["selected_room"] = "ROOM1"
        state["screen"] = "CONFIRM_ACK"
        return True
    if edge == "select_room2":
        state["selected_room"] = "ROOM2"
        state["screen"] = "CONFIRM_ACK"
        return True
    if edge == "delivery_confirmed":
        if state["selected_room"]:
            append_log_line(state["selected_room"])
        if state["active_room"] in state["pending_rooms"]:
            state["pending_rooms"].remove(state["active_room"])
        state["active_room"] = None
        state["selected_room"] = None
        if not _set_active_from_queue():
            state["mode"] = "DOCK_IDLE"
            state["screen"] = "HOME"
        return True

    return False
