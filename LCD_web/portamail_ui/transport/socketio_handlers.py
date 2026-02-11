from interface.event_store import add_event, read_events
from state.model import set_bit, snapshot_state, set_events
from state.state_machine import handle_edge


def register_socketio(socketio):
    def emit_state():
        set_events(read_events())
        socketio.emit("state", snapshot_state())

    def emit_events():
        socketio.emit("events", read_events())

    @socketio.on("connect")
    def on_connect(auth):
        emit_state()
        emit_events()

    @socketio.on("press")
    def on_press(data):
        bit = (data or {}).get("bit")
        if bit and set_bit(bit, True):
            emit_state()

    @socketio.on("release")
    def on_release(data):
        payload = data or {}
        bit = payload.get("bit")
        edge = payload.get("edge")
        if bit and set_bit(bit, False):
            emit_state()
        if edge:
            add_event(edge)
            emit_events()
            if handle_edge(edge):
                emit_state()

    return emit_state, emit_events
