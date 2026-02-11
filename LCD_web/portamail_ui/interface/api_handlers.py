from flask import jsonify, request
from interface.event_store import read_events
from interface.validators import validate_mode
from state.model import snapshot_state, set_events
from state.state_machine import set_mode


def register_api(app, emit_state):
    @app.route("/api/state")
    def api_state():
        set_events(read_events())
        return jsonify(snapshot_state())

    @app.route("/api/events")
    def api_events():
        return jsonify(read_events())

    @app.route("/api/mode", methods=["POST"])
    def api_mode():
        data = request.get_json(silent=True) or {}
        mode = data.get("mode")
        if not validate_mode(mode):
            return jsonify({"ok": False, "error": "unknown_mode"}), 400
        set_mode(mode)
        emit_state()
        return jsonify({"ok": True, "mode": mode})
