MODES = {"DOCK_IDLE", "ARRIVED"}
SCREENS = {
    "HOME",
    "ARRIVED",
    "CONFIRM_SELECT",
    "CONFIRM_ACK",
    "DELIVERING_ROOM1",
    "DELIVERING_ROOM2",
}

BIT_KEYS = {
    "room1_start_pressed",
    "room2_start_pressed",
    "power_pressed",
    "confirm_package_pressed",
    "confirm_room1_pressed",
    "confirm_room2_pressed",
    "package_confirmed_pressed",
}

EDGE_EVENTS = {
    "start_room1",
    "start_room2",
    "power_edge",
    "open_confirm_flow",
    "select_room1",
    "select_room2",
    "delivery_confirmed",
}


def make_event(ts, name, payload=None):
    data = payload or {}
    # Backward-compatible: keep name/payload and also include type/data.
    return {
        "ts": ts,
        "name": name,
        "payload": data,
        "type": name,
        "data": data,
    }
