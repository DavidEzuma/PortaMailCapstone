from interface.contract import MODES


def validate_mode(mode):
    return mode in MODES


def validate_edge_for_screen(edge, screen, mode, selected_room):
    if edge in {"select_mapping", "select_navigation"}:
        return screen == "MODE_SELECT"
    if edge in {"save_map", "save_map_open"}:
        return mode == "MAPPING"
    if edge in {"save_location_room1", "save_location_room2", "save_location_origin", "save_location_none"}:
        return screen == "SAVE_MAP_SELECT"
    if edge == "open_confirm_flow":
        return mode == "ARRIVED" or screen == "ARRIVED"
    if edge in {"select_room1", "select_room2"}:
        return screen == "CONFIRM_SELECT"
    if edge == "delivery_confirmed":
        return screen == "CONFIRM_ACK" and selected_room is not None
    if edge == "go_back":
        return screen in {"SAVE_MAP_SELECT", "MAPPING", "HOME"}
    if edge == "start_origin":
        return screen == "HOME"
    return True
