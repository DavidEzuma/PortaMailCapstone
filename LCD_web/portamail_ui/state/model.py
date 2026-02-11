from interface.contract import BIT_KEYS

_state = {
    "mode": "DOCK_IDLE",
    "screen": "HOME",
    "selected_room": None,
    "pending_rooms": [],
    "active_room": None,
    "bits": {key: 0 for key in BIT_KEYS},
    "events": [],
}


def get_state():
    return _state


def snapshot_state():
    return {
        "mode": _state["mode"],
        "screen": _state["screen"],
        "selected_room": _state["selected_room"],
        "pending_rooms": list(_state["pending_rooms"]),
        "active_room": _state["active_room"],
        "bits": dict(_state["bits"]),
        "events": list(_state["events"]),
    }


def set_bit(bit, value):
    if bit in _state["bits"]:
        _state["bits"][bit] = 1 if value else 0
        return True
    return False


def set_events(events):
    _state["events"] = list(events)
