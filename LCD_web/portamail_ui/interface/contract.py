MODES = {"DOCK_IDLE", "ARRIVED", "MAPPING"}
SCREENS = {
    "MODE_SELECT",
    "HOME",
    "ARRIVED",
    "CONFIRM_SELECT",
    "CONFIRM_ACK",
    "DELIVERING_ROOM1",
    "DELIVERING_ROOM2",
    "MAPPING",
    "SAVE_MAP_SELECT",
    "SAVE_LOCATION_SELECT",
    "PROCESSING",
}

BIT_KEYS = {
    "room1_start_pressed",
    "room2_start_pressed",
    "power_pressed",
    "confirm_package_pressed",
    "confirm_room1_pressed",
    "confirm_room2_pressed",
    "package_confirmed_pressed",
    "select_mapping_pressed",
    "select_navigation_pressed",
    "save_map_pressed",
    "save_location_room1_pressed",
    "save_location_room2_pressed",
    "save_location_origin_pressed",
    "save_location_none_pressed",
    "start_origin_pressed",
    "back_pressed",
    "save_location_btn_pressed",
    "save_map_now_pressed",
    "mark_location_room1_pressed",
    "mark_location_room2_pressed",
    "mark_location_origin_pressed",
}

EDGE_EVENTS = {
    "start_room1",
    "start_room2",
    "start_origin",
    "power_edge",
    "open_confirm_flow",
    "select_room1",
    "select_room2",
    "delivery_confirmed",
    "select_mapping",
    "select_navigation",
    "save_map",
    "save_map_open",
    "save_location_room1",
    "save_location_room2",
    "save_location_origin",
    "save_location_none",
    "go_back",
    "save_location_open",
    "mark_location_room1",
    "mark_location_room2",
    "mark_location_origin",
    "cancel_save_location",
    "save_map_now",
    "map_saved",
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
