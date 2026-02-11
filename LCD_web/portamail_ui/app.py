# Run:
# python3 -m venv venv
# source venv/bin/activate
# pip install -r requirements.txt
# python app.py
# Open http://127.0.0.1:5050

from flask import Flask, render_template
from flask_socketio import SocketIO
from interface.api_handlers import register_api
from state.state_machine import ensure_log_dir
from transport.socketio_handlers import register_socketio

app = Flask(__name__)
app.config["SECRET_KEY"] = "portamail-lcd-phase2"
socketio = SocketIO(app, cors_allowed_origins="*")


@app.route("/")
def index():
    return render_template("index.html")


def main():
    ensure_log_dir()
    emit_state, _emit_events = register_socketio(socketio)
    register_api(app, emit_state)
    socketio.run(app, host="0.0.0.0", port=5050, debug=True)


if __name__ == "__main__":
    main()
