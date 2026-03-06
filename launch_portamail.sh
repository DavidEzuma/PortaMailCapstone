#!/bin/bash
# PortaMail top-level startup script.
#
# 1. Starts the LCD Flask server (touchscreen UI).
# 2. Opens Chromium in kiosk mode on the connected display.
# 3. Waits for the user to select Mapping or Navigation mode on the screen.
# 4. Launches the appropriate ROS 2 stack.
#
# Usage:
#   ./launch_portamail.sh
#   ./launch_portamail.sh --lcd-url http://127.0.0.1:5050

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LCD_DIR="${SCRIPT_DIR}/LCD_web/portamail_ui"
LCD_URL="http://127.0.0.1:5050"

# Parse optional --lcd-url argument
while [[ $# -gt 0 ]]; do
    case "$1" in
        --lcd-url) LCD_URL="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

# Source workspace (prefer built install, fall back to system ROS)
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
# 1. Start the LCD Flask server
# ---------------------------------------------------------------------------
echo "=========================================="
echo "  PortaMail Startup"
echo "=========================================="
echo ""
echo "[startup] Starting LCD server at ${LCD_URL} ..."

cd "${LCD_DIR}"

if [[ ! -d venv ]]; then
    echo "[startup] Creating Python venv for LCD server ..."
    python3 -m venv venv
    venv/bin/pip install -r requirements.txt -q
fi

venv/bin/python app.py &
LCD_PID=$!
cd "${SCRIPT_DIR}"

# Give Flask time to bind before opening the browser
sleep 2

echo "[startup] LCD server running (PID ${LCD_PID})"

# ---------------------------------------------------------------------------
# 2. Open Chromium in kiosk mode
# ---------------------------------------------------------------------------
echo "[startup] Opening kiosk display ..."

# Use the current session's DISPLAY, or default to :0
DISPLAY="${DISPLAY:-:0}" \
XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}" \
"${CHROMIUM}" \
    --kiosk \
    --no-sandbox \
    --disable-infobars \
    --disable-session-crashed-bubble \
    --disable-restore-session-state \
    --noerrdialogs \
    "${LCD_URL}" &
BROWSER_PID=$!

echo "[startup] Browser running (PID ${BROWSER_PID})"
echo ""
echo "  Select a mode on the touchscreen to continue."
echo ""

# ---------------------------------------------------------------------------
# 3. Wait for mode selection
# ---------------------------------------------------------------------------
MODE=""
while [[ -z "${MODE}" ]]; do
    EVENTS=$(curl -sf "${LCD_URL}/api/events" 2>/dev/null || echo "[]")
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

# ---------------------------------------------------------------------------
# 4. Launch appropriate ROS 2 stack
# ---------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "[startup] Shutting down ..."
    kill "${BROWSER_PID}" 2>/dev/null || true
    kill "${LCD_PID}"     2>/dev/null || true
    [[ -n "${MAP_PID:-}"   ]] && kill "${MAP_PID}"   2>/dev/null || true
    [[ -n "${COORD_PID:-}" ]] && kill "${COORD_PID}" 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

if [[ "${MODE}" == "mapping" ]]; then
    echo "[startup] Launching: hardware + SLAM Toolbox + joystick (real LiDAR)"
    ros2 launch portamail_navigator mapping.launch.py \
        use_mock_driver:=false \
        use_real_lidar:=true &
    MAP_PID=$!

    # Give the hardware stack a moment before starting the coordinator
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

echo "[startup] All processes running. Press Ctrl+C to stop."
echo ""

wait
