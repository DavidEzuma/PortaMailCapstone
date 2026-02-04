# Run:
# python3 -m venv venv
# source venv/bin/activate
# pip install -r requirements.txt
# python app.py
# Open http://127.0.0.1:5050

from datetime import datetime
from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
app.config["SECRET_KEY"] = "portamail-lcd-step1"
socketio = SocketIO(app, cors_allowed_origins="*")

state = {
    "mode": "DOCK_IDLE",
    "screen": "HOME",
    "selected_room": None,
    "bits": {
        "room1_start_pressed": 0,
        "room2_start_pressed": 0,
        "power_pressed": 0,
    },
}

EVENTS = []
EVENT_LIMIT = 50


def emit_state():
    socketio.emit("state", state)


def emit_events():
    socketio.emit("events", EVENTS[-EVENT_LIMIT:])


def add_event(name, payload=None):
    evt = {
        "ts": datetime.now().isoformat(),
        "name": name,
        "payload": payload or {},
    }
    EVENTS.append(evt)
    if len(EVENTS) > EVENT_LIMIT:
        del EVENTS[0 : len(EVENTS) - EVENT_LIMIT]
    emit_events()


def set_bit(bit, value):
    if bit in state["bits"]:
        state["bits"][bit] = 1 if value else 0
        emit_state()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/state")
def api_state():
    return jsonify(state)


@app.route("/api/events")
def api_events():
    return jsonify(EVENTS[-EVENT_LIMIT:])


@socketio.on("connect")
def on_connect(auth):
    emit_state()
    emit_events()


@socketio.on("press")
def on_press(data):
    bit = (data or {}).get("bit")
    if bit:
        set_bit(bit, True)


@socketio.on("release")
def on_release(data):
    payload = data or {}
    bit = payload.get("bit")
    edge = payload.get("edge")
    if bit:
        set_bit(bit, False)
    if edge:
        add_event(edge)


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5050, debug=True)
