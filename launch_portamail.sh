#!/bin/bash
# PortaMail top-level startup script.
#
# 1. Starts the LCD Flask server (touchscreen UI).
# 2. Opens Chromium in kiosk mode on the connected display.
# 3. Waits for the user to select Mapping or Navigation mode on the screen.
# 4. Launches the appropriate ROS 2 stack.
# 5. Monitors for the Back button — when the UI returns to MODE_SELECT the ROS
#    stack is stopped and the mode-selection loop restarts (LiDAR will re-spin).
#
# Usage:
#   ./launch_portamail.sh
#   ./launch_portamail.sh --lcd-url http://127.0.0.1:5050

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LCD_DIR="${SCRIPT_DIR}/LCD_web/portamail_ui"
LCD_URL="http://127.0.0.1:5050"

# Parse optional arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --lcd-url) LCD_URL="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

# ---------------------------------------------------------------------------
# Source ROS 2 workspace
# ---------------------------------------------------------------------------
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/install/setup.bash"
else
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

# ---------------------------------------------------------------------------
# Locate Chromium binary
# ---------------------------------------------------------------------------
if command -v chromium-browser &>/dev/null; then
    CHROMIUM="chromium-browser"
elif command -v chromium &>/dev/null; then
    CHROMIUM="chromium"
else
    echo "[startup] ERROR: Chromium not found. Install it with:"
    echo "  sudo apt install chromium-browser"
    exit 1
fi

# ---------------------------------------------------------------------------
echo "=========================================="
echo "  PortaMail Startup"
echo "=========================================="
echo ""

# ---------------------------------------------------------------------------
# 1. Start the LCD Flask server
# ---------------------------------------------------------------------------
echo "[startup] Starting LCD server at ${LCD_URL} ..."

cd "${LCD_DIR}"

if [[ ! -f venv/bin/pip ]]; then
    echo "[startup] Creating Python venv ..."
    rm -rf venv
    python3 -m venv venv
fi

venv/bin/pip install -r requirements.txt -q

venv/bin/python app.py &
LCD_PID=$!
cd "${SCRIPT_DIR}"

sleep 2
echo "[startup] LCD server running (PID ${LCD_PID})"

# ---------------------------------------------------------------------------
# 2. Detect the graphical session display (wait up to 60 s for auto-login)
# ---------------------------------------------------------------------------
UID_NUM=$(id -u)
CHROMIUM_EXTRA_FLAGS=""
DISPLAY_FOUND=false

echo "[startup] Waiting for graphical session ..."
for _i in $(seq 1 60); do
    _WSOCK=$(ls "/run/user/${UID_NUM}/wayland-"* 2>/dev/null | head -1 || true)
    if [[ -S "${_WSOCK:-}" ]]; then
        export XDG_RUNTIME_DIR="/run/user/${UID_NUM}"
        export WAYLAND_DISPLAY
        WAYLAND_DISPLAY=$(basename "${_WSOCK}")
        if [[ -S "${XDG_RUNTIME_DIR}/bus" ]]; then
            export DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"
        fi
        CHROMIUM_EXTRA_FLAGS="--ozone-platform=wayland"
        DISPLAY_FOUND=true
        echo "[startup] Display: Wayland (${WAYLAND_DISPLAY})"
        break
    fi

    _XSOCK=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -1 || true)
    if [[ -S "${_XSOCK:-}" ]]; then
        export DISPLAY=":$(basename "${_XSOCK}" | tr -d 'X')"
        export XAUTHORITY="${HOME}/.Xauthority"
        DISPLAY_FOUND=true
        echo "[startup] Display: X11 (${DISPLAY})"
        break
    fi

    sleep 1
done

if [[ "${DISPLAY_FOUND}" == false ]]; then
    echo ""
    echo "[startup] ERROR: No graphical session found after 60 s."
    kill "${LCD_PID}" 2>/dev/null || true
    exit 1
fi

# ---------------------------------------------------------------------------
# 3. Disable screen blanking and hide mouse cursor
# ---------------------------------------------------------------------------
if [[ "${DISPLAY_FOUND}" == true ]]; then
    echo "[startup] Disabling screen blanking ..."
    if command -v xset &>/dev/null && [[ -n "${DISPLAY:-}" ]]; then
        xset s off
        xset -dpms
        xset s noblank
    fi
fi

if command -v unclutter &>/dev/null; then
    unclutter -idle 0 -root &
fi

# ---------------------------------------------------------------------------
# 4. Open Chromium in kiosk mode (once — persists for entire session)
# ---------------------------------------------------------------------------
echo "[startup] Opening kiosk display ..."

# shellcheck disable=SC2086
"${CHROMIUM}" \
    ${CHROMIUM_EXTRA_FLAGS} \
    --kiosk \
    --no-sandbox \
    --disable-infobars \
    --disable-session-crashed-bubble \
    --disable-restore-session-state \
    --noerrdialogs \
    "${LCD_URL}" &
BROWSER_PID=$!

echo "[startup] Browser running (PID ${BROWSER_PID})"

# ---------------------------------------------------------------------------
# Cleanup on Ctrl-C / SIGTERM
# ---------------------------------------------------------------------------
MAP_PID=""
COORD_PID=""

cleanup() {
    echo ""
    echo "[startup] Shutting down ..."
    _kill_ros
    kill "${BROWSER_PID}" 2>/dev/null || true
    kill "${LCD_PID}"     2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

# ---------------------------------------------------------------------------
# Helper: get the last event timestamp from the LCD API
# ---------------------------------------------------------------------------
_get_last_ts() {
    curl -sf "${LCD_URL}/api/events" 2>/dev/null \
        | python3 -c "
import sys, json
evts = json.load(sys.stdin)
print(evts[-1]['ts'] if evts else '')
" 2>/dev/null || echo ""
}

# ---------------------------------------------------------------------------
# Helper: kill the ROS stack (mapping + coordinator)
# ---------------------------------------------------------------------------
_kill_ros() {
    echo "[startup] Stopping ROS stack ..."
    # Disable exit-on-error inside this function so cleanup always completes
    set +e

    # Step 1: Signal sllidar_node directly so it runs its motor-stop shutdown
    # handler before anything else. ros2 launch does NOT forward SIGINT to
    # children when the launch process itself is signalled from outside the
    # terminal foreground process group.
    pkill -SIGINT -f sllidar_node 2>/dev/null || true
    sleep 2

    # Step 2: Signal the ros2 launch processes directly. ros2 launch will
    # propagate SIGINT to its remaining children (SLAM, EKF, foxglove, etc.).
    [[ -n "${MAP_PID}"   ]] && kill -SIGINT "${MAP_PID}"   2>/dev/null || true
    [[ -n "${COORD_PID}" ]] && kill -SIGINT "${COORD_PID}" 2>/dev/null || true

    # Step 3: Give nodes up to 5 s to shut down cleanly
    sleep 5

    # Step 4: Force-kill any survivors
    pkill -SIGKILL -f sllidar_node 2>/dev/null || true
    [[ -n "${MAP_PID}"   ]] && kill -SIGKILL "${MAP_PID}"   2>/dev/null || true
    [[ -n "${COORD_PID}" ]] && kill -SIGKILL "${COORD_PID}" 2>/dev/null || true
    [[ -n "${MAP_PID}"   ]] && wait "${MAP_PID}"   2>/dev/null || true
    [[ -n "${COORD_PID}" ]] && wait "${COORD_PID}" 2>/dev/null || true
    MAP_PID=""
    COORD_PID=""
    set -e
    echo "[startup] ROS stack stopped."
}

# ---------------------------------------------------------------------------
# 5. Mode-selection + ROS launch loop
#    Restarts whenever the Back button returns the UI to MODE_SELECT.
# ---------------------------------------------------------------------------
SINCE_TS=""   # initialise; updated before each _kill_ros call

while true; do
    echo ""
    echo "  Select a mode on the touchscreen to continue."
    echo ""

    # Wait for mode selection
    MODE=""
    while [[ -z "${MODE}" ]]; do
        if [[ -n "${SINCE_TS}" ]]; then
            EVENTS=$(curl -sf "${LCD_URL}/api/events?since_ts=$(python3 -c "import urllib.parse,sys; print(urllib.parse.quote('${SINCE_TS}', safe=''))")" 2>/dev/null || echo "[]")
        else
            EVENTS=$(curl -sf "${LCD_URL}/api/events" 2>/dev/null || echo "[]")
        fi

        if echo "${EVENTS}" | python3 -c "
import sys, json
evts = json.load(sys.stdin)
exit(0 if any(e.get('name') == 'select_mapping' for e in evts) else 1)
" 2>/dev/null; then
            MODE="mapping"
        elif echo "${EVENTS}" | python3 -c "
import sys, json
evts = json.load(sys.stdin)
exit(0 if any(e.get('name') == 'select_navigation' for e in evts) else 1)
" 2>/dev/null; then
            MODE="navigation"
        else
            sleep 1
        fi
    done

    echo "[startup] Mode selected: ${MODE}"
    echo ""

    # Launch appropriate ROS 2 stack
    if [[ "${MODE}" == "mapping" ]]; then
        echo "[startup] Launching: hardware + SLAM Toolbox + joystick (real LiDAR)"
        ros2 launch portamail_navigator mapping.launch.py \
            use_mock_driver:=true \
            use_real_lidar:=true &
        MAP_PID=$!

        sleep 3

        echo "[startup] Launching: coordinator + LCD bridge (mapping mode)"
        ros2 launch portamail_coordinator bringup.launch.py \
            mode:=mapping \
            lcd_url:="${LCD_URL}" &
        COORD_PID=$!

    else
        echo "[startup] Launching: coordinator + LCD bridge (navigation mode)"
        ros2 launch portamail_coordinator bringup.launch.py \
            mode:=navigation \
            lcd_url:="${LCD_URL}" &
        COORD_PID=$!
    fi

    echo "[startup] All processes running. Press Back on the touchscreen to return to mode select."
    echo ""

    # Monitor: wait for Back button (screen returns to MODE_SELECT) or process exit
    while true; do
        sleep 2

        # Check if UI went back to mode selection
        SCREEN=$(curl -sf "${LCD_URL}/api/state" 2>/dev/null \
            | python3 -c "import sys,json; print(json.load(sys.stdin).get('screen',''))" 2>/dev/null || echo "")

        if [[ "${SCREEN}" == "MODE_SELECT" ]]; then
            echo "[startup] Back button detected — stopping ROS stack and restarting mode selection."
            # Snapshot the event log NOW (before the ~7 s kill window) so any
            # mode-select tap the user makes during shutdown is not missed.
            SINCE_TS=$(_get_last_ts)
            _kill_ros
            break
        fi

        # Also break if both processes have exited on their own
        MAP_ALIVE=false
        COORD_ALIVE=false
        [[ -n "${MAP_PID}"   ]] && kill -0 "${MAP_PID}"   2>/dev/null && MAP_ALIVE=true
        [[ -n "${COORD_PID}" ]] && kill -0 "${COORD_PID}" 2>/dev/null && COORD_ALIVE=true

        if [[ "${MAP_ALIVE}" == false && "${COORD_ALIVE}" == false ]]; then
            echo "[startup] All ROS processes exited — returning to mode selection."
            SINCE_TS=$(_get_last_ts)
            MAP_PID=""
            COORD_PID=""
            break
        fi
    done
done
