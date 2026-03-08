#!/bin/bash
# PortaMail top-level startup script.
#
# 1. Starts the LCD Flask server (touchscreen UI).
# 2. Opens Chromium in kiosk mode on the connected display.
# 3. Waits for the user to select Mapping or Navigation mode on the screen.
# 4. Launches the appropriate ROS 2 stack.
#
# Prerequisites:
#   - GDM auto-login must be enabled so a graphical session is running:
#       sudo nano /etc/gdm3/custom.conf
#       # Under [daemon]:
#       AutomaticLoginEnable=true
#       AutomaticLogin=davidezuma
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
    # Prefer Wayland — check for any wayland-N socket in the user runtime dir
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

    # Fall back to X11 — check /tmp/.X11-unix/
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
    echo "  Make sure GDM auto-login is enabled in /etc/gdm3/custom.conf:"
    echo "    [daemon]"
    echo "    AutomaticLoginEnable=true"
    echo "    AutomaticLogin=davidezuma"
    echo ""
    echo "  Alternatively, run this script from the GNOME Terminal on the Pi"
    echo "  (not from SSH)."
    kill "${LCD_PID}" 2>/dev/null || true
    exit 1
fi

# ---------------------------------------------------------------------------
# 3. Open Chromium in kiosk mode
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
echo ""
echo "  Select a mode on the touchscreen to continue."
echo ""

# ---------------------------------------------------------------------------
# 4. Wait for mode selection
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
# 5. Launch appropriate ROS 2 stack
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
