from interface.contract import MODES


def validate_mode(mode):
    return mode in MODES


def validate_edge_for_screen(edge, screen, mode, selected_room):
    if edge == "open_confirm_flow":
        return mode == "ARRIVED" or screen == "ARRIVED"
    if edge in {"select_room1", "select_room2"}:
        return screen == "CONFIRM_SELECT"
    if edge == "delivery_confirmed":
        return screen == "CONFIRM_ACK" and selected_room is not None
    return True
