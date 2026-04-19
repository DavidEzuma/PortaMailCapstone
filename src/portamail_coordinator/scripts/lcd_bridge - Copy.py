#!/usr/bin/env python3
"""
lcd_bridge — ROS 2 node that bridges the PortaMail LCD Flask UI to the
navigation_coordinator node.

LCD events  →  user_delivery_request topic
system_status topic  →  POST /api/mode to LCD server

In mapping mode:
  save_location_room1/room2/origin → TF lookup → writes locations.yaml
  save_map_now / go_back → map file management

In navigation mode:
  start_room1 / start_room2 → builds delivery queue, starts navigation
  delivery_confirmed        → advance queue or return to mailroom
  system_status JSON        → drive LCD state (ARRIVED / DOCK_IDLE)

Delivery state machine:
  IDLE             – waiting for a start_room event
  NAVIGATING       – Nav2 goal sent, waiting for ARRIVED status
  WAITING_CONFIRM  – robot at destination, waiting for delivery_confirmed
  RETURNING        – navigating back to mailroom (no confirm needed)

Delivery queue: ordered list of location keys, e.g. ["office_101", "mailroom"].
The mailroom stop is always last and skips the WAITING_CONFIRM step (robot just
docks without needing human confirmation).

Persistence: state is written atomically to ~/.portamail_delivery_state.json on
every transition so a crash mid-delivery can be recovered on restart.
"""

import glob
import json
import math
import os
import tempfile
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import tf2_ros
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_ROOM_TO_LOCATION = {
    "ROOM1":  "office_101",
    "ROOM2":  "office_102",
    "ORIGIN": "mailroom",
}

_SAVE_LOCATION_MAP = {
    "save_location_room1":  "office_101",
    "save_location_room2":  "office_102",
    "save_location_origin": "mailroom",
}

_MARK_LOCATION_MAP = {
    "mark_location_room1":  "office_101",
    "mark_location_room2":  "office_102",
    "mark_location_origin": "mailroom",
}

_IDLE            = "IDLE"
_NAVIGATING      = "NAVIGATING"
_WAITING_CONFIRM = "WAITING_CONFIRM"
_RETURNING       = "RETURNING"

_STATE_FILE = os.path.expanduser("~/.portamail_delivery_state.json")
_STATE_SCHEMA_VERSION = 1

# ---------------------------------------------------------------------------


class LcdBridge(Node):
    def __init__(self):
        super().__init__("lcd_bridge")

        self.declare_parameter("lcd_url",             "http://127.0.0.1:5050")
        self.declare_parameter("poll_hz",             2.0)
        self.declare_parameter("ros_mode",            "navigation")
        self.declare_parameter("locations_yaml_path", "")
        self.declare_parameter(
            "maps_dir",
            os.path.expanduser("~/PortaMailCapstone/maps"),
        )

        self._lcd_url  = self.get_parameter("lcd_url").get_parameter_value().string_value
        poll_hz        = self.get_parameter("poll_hz").get_parameter_value().double_value
        self._ros_mode = self.get_parameter("ros_mode").get_parameter_value().string_value
        self._loc_yaml = self.get_parameter("locations_yaml_path").get_parameter_value().string_value
        self._maps_dir = self.get_parameter("maps_dir").get_parameter_value().string_value

        self._pub = self.create_publisher(String, "user_delivery_request", 10)
        self.create_subscription(String, "system_status", self._on_status, 10)

        # Delivery state
        self._bridge_state:    str       = _IDLE
        self._delivery_queue:  list[str] = []
        self._current_dest:    str | None = None

        # LCD event dedup
        self._seen:    dict[str, bool] = {}
        self._last_ts: str | None      = None

        # TF2 (used in mapping mode to look up robot pose)
        if _TF2_AVAILABLE:
            self._tf_buffer   = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self.get_logger().warn(
                "tf2_ros not available — save_location will not record TF pose"
            )

        self._drain_history()

        # Recovery: if a delivery was in progress when we last crashed, restore
        # it and re-issue the goal after Nav2 has had time to start up.
        self._recovery_timer = None
        self._load_persisted_state()

        self.create_timer(1.0 / poll_hz, self._poll)
        self.get_logger().info(
            f"LCD bridge ready | url={self._lcd_url} | ros_mode={self._ros_mode}"
        )

    # -----------------------------------------------------------------------
    # HTTP helpers
    # -----------------------------------------------------------------------

    def _http_get(self, path: str):
        url = f"{self._lcd_url}{path}"
        with urllib.request.urlopen(url, timeout=3) as resp:
            return json.loads(resp.read())

    def _http_post(self, path: str, body: dict):
        data = json.dumps(body).encode()
        req  = urllib.request.Request(
            f"{self._lcd_url}{path}", data=data, method="POST"
        )
        req.add_header("Content-Type", "application/json")
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read())

    # -----------------------------------------------------------------------
    # Startup: drain history so old events are not replayed
    # -----------------------------------------------------------------------

    def _drain_history(self):
        try:
            events = self._http_get("/api/events")
            for evt in events:
                key = f"{evt.get('ts')}|{evt.get('name')}"
                self._seen[key] = True
            if events:
                self._last_ts = events[-1].get("ts")
            self.get_logger().info(f"Drained {len(events)} historical LCD events")
        except Exception as exc:
            self.get_logger().warn(f"Could not drain LCD history: {exc}")

    # -----------------------------------------------------------------------
    # Polling timer
    # -----------------------------------------------------------------------

    def _poll(self):
        try:
            path = "/api/events"
            if self._last_ts:
                path += "?since_ts=" + urllib.parse.quote(self._last_ts, safe="")
            events = self._http_get(path)
        except Exception as exc:
            self.get_logger().warn(f"LCD poll error: {exc}")
            return

        for evt in events:
            key = f"{evt.get('ts')}|{evt.get('name')}"
            if key in self._seen:
                continue
            self._seen[key] = True
            if len(self._seen) > 500:
                for k in list(self._seen)[:200]:
                    del self._seen[k]
            self._last_ts = evt.get("ts") or self._last_ts
            self._handle_event(evt.get("name"))

    # -----------------------------------------------------------------------
    # Event dispatch
    # -----------------------------------------------------------------------

    def _handle_event(self, name: str | None):
        if not name:
            return
        self.get_logger().debug(f"LCD event={name!r}  bridge={self._bridge_state}")

        # --- Mapping mode ---
        if self._ros_mode == "mapping":
            if name == "save_map":
                self._publish("save_map")
            elif name in _SAVE_LOCATION_MAP:
                self._save_location_and_map(name)
            elif name == "save_location_none":
                self._delete_all_maps()
                self._publish("save_map")
            elif name in _MARK_LOCATION_MAP:
                self._mark_location(name)
            elif name == "save_map_now":
                self._delete_all_maps()
                self._publish("save_map")
            elif name == "go_back":
                self._delete_all_maps()
                self._clear_saved_locations()
            return

        # --- Navigation mode ---
        if name == "start_room1" and self._bridge_state == _IDLE:
            self._start_delivery(["office_101", "mailroom"])

        elif name == "start_room2" and self._bridge_state == _IDLE:
            self._start_delivery(["office_102", "mailroom"])

        elif name == "start_origin" and self._bridge_state == _IDLE:
            self._start_delivery(["mailroom"])

        elif name == "start_room2" and self._bridge_state == _NAVIGATING:
            # Late addition: insert room2 before the mailroom return leg
            if "office_102" not in self._delivery_queue:
                try:
                    idx = self._delivery_queue.index("mailroom")
                    self._delivery_queue.insert(idx, "office_102")
                    self._persist_state()
                    self.get_logger().info(
                        f"Queued office_102 mid-route. Queue: {self._delivery_queue}"
                    )
                except ValueError:
                    self._delivery_queue.append("office_102")
                    self._persist_state()

        elif name == "delivery_confirmed" and self._bridge_state == _WAITING_CONFIRM:
            self._advance_queue()

    # -----------------------------------------------------------------------
    # Delivery queue helpers
    # -----------------------------------------------------------------------

    def _start_delivery(self, queue: list[str]):
        self._delivery_queue = queue.copy()
        dest = self._delivery_queue.pop(0)
        self._current_dest  = dest
        self._bridge_state  = _NAVIGATING
        self._persist_state()
        self._navigate_to(dest)

    def _advance_queue(self):
        if self._delivery_queue:
            dest = self._delivery_queue.pop(0)
            self._current_dest = dest
            if dest == "mailroom":
                # Return leg — no confirm needed
                self._bridge_state = _RETURNING
            else:
                self._bridge_state = _NAVIGATING
            self._persist_state()
            self._navigate_to(dest)
        else:
            # Queue exhausted — go home
            self._current_dest = "mailroom"
            self._bridge_state = _RETURNING
            self._persist_state()
            self._navigate_to("mailroom")

    # -----------------------------------------------------------------------
    # system_status subscriber — parses JSON from navigation_coordinator
    # -----------------------------------------------------------------------

    def _on_status(self, msg: String):
        text = msg.data
        self.get_logger().debug(f"system_status={text!r}  bridge={self._bridge_state}")

        # Parse JSON envelope; fall back gracefully for legacy builds during
        # the transition period where coordinator may still emit plain strings.
        try:
            parsed = json.loads(text)
            msg_type = parsed.get("type", "")
            code     = parsed.get("code", "")
            detail   = parsed.get("detail", "")
        except (json.JSONDecodeError, AttributeError):
            # Legacy fallback
            if "Arrived" in text or "Arrived" in text:
                msg_type, code, detail = "status", "ARRIVED", ""
            elif "Map Saved" in text:
                msg_type, code, detail = "status", "MAP_SAVED", ""
            elif "Error" in text:
                msg_type, code, detail = "error", "NAV_FAILED", text
            else:
                return

        if self._ros_mode == "navigation":
            if msg_type == "status" and code == "ARRIVED":
                if self._bridge_state == _NAVIGATING:
                    if self._current_dest == "mailroom":
                        # Should not happen (RETURNING handles mailroom arrival)
                        # but guard for robustness
                        self._post_lcd_mode("DOCK_IDLE")
                        self._bridge_state = _IDLE
                        self._clear_persisted_state()
                    else:
                        self._post_lcd_mode("ARRIVED")
                        self._bridge_state = _WAITING_CONFIRM
                        self._persist_state()
                elif self._bridge_state == _RETURNING:
                    self._post_lcd_mode("DOCK_IDLE")
                    self._bridge_state = _IDLE
                    self._current_dest = None
                    self._delivery_queue = []
                    self._clear_persisted_state()

            elif msg_type == "error" and self._bridge_state in (_NAVIGATING, _RETURNING):
                self.get_logger().warn(
                    f"Nav error ({code}: {detail}) — returning to mailroom"
                )
                # Show the error screen on the LCD before queuing the return trip.
                human_msg = self._nav_error_human_message(code, detail)
                try:
                    self._http_post("/api/edge", {"edge": "nav_error",
                                                  "payload": {"message": human_msg}})
                except Exception as exc:
                    self.get_logger().warn(f"Could not POST nav_error to LCD: {exc}")
                    self._post_lcd_mode("DOCK_IDLE")  # fallback
                self._delivery_queue = []
                self._current_dest   = "mailroom"
                self._bridge_state   = _RETURNING
                self._persist_state()
                self._navigate_to("mailroom")

        elif self._ros_mode == "mapping":
            if msg_type == "status" and code == "MAP_SAVED":
                self.get_logger().info("Map saved — signalling UI")
                try:
                    self._http_post("/api/edge", {"edge": "map_saved"})
                except Exception as exc:
                    self.get_logger().warn(f"Could not POST map_saved to LCD: {exc}")

    # -----------------------------------------------------------------------
    # Persistence
    # -----------------------------------------------------------------------

    def _persist_state(self):
        """Atomically write delivery state to disk."""
        state = {
            "schema_version": _STATE_SCHEMA_VERSION,
            "bridge_state":   self._bridge_state,
            "current_dest":   self._current_dest,
            "delivery_queue": self._delivery_queue,
        }
        tmp_path = _STATE_FILE + ".tmp"
        try:
            with open(tmp_path, "w") as f:
                json.dump(state, f)
            os.replace(tmp_path, _STATE_FILE)
        except Exception as exc:
            self.get_logger().warn(f"Could not persist delivery state: {exc}")

    def _clear_persisted_state(self):
        try:
            if os.path.exists(_STATE_FILE):
                os.remove(_STATE_FILE)
        except Exception as exc:
            self.get_logger().warn(f"Could not clear persisted state: {exc}")

    def _load_persisted_state(self):
        """On startup, restore an interrupted delivery if one was in progress."""
        if self._ros_mode != "navigation":
            return
        if not os.path.exists(_STATE_FILE):
            return
        try:
            with open(_STATE_FILE, "r") as f:
                state = json.load(f)
            if state.get("schema_version") != _STATE_SCHEMA_VERSION:
                self.get_logger().warn(
                    "Persisted state schema mismatch — discarding."
                )
                self._clear_persisted_state()
                return
            bridge_state = state.get("bridge_state", _IDLE)
            if bridge_state == _IDLE:
                self._clear_persisted_state()
                return
            self._bridge_state   = bridge_state
            self._current_dest   = state.get("current_dest")
            self._delivery_queue = state.get("delivery_queue", [])
            self.get_logger().warn(
                f"Recovering delivery: state={self._bridge_state} "
                f"dest={self._current_dest} queue={self._delivery_queue}"
            )
            if bridge_state == _NAVIGATING:
                # Re-issue the nav goal after Nav2 has had time to start (10 s).
                self._recovery_timer = self.create_timer(10.0, self._recovery_nav)
            elif bridge_state == _WAITING_CONFIRM:
                # Robot presumably already at destination — re-post ARRIVED.
                self._recovery_timer = self.create_timer(3.0, self._recovery_confirm)
            elif bridge_state == _RETURNING:
                self._recovery_timer = self.create_timer(10.0, self._recovery_nav)
        except Exception as exc:
            self.get_logger().warn(
                f"Could not load persisted state ({exc}) — starting IDLE."
            )
            self._clear_persisted_state()

    def _recovery_nav(self):
        """One-shot timer: re-send the in-progress navigation goal."""
        if self._recovery_timer:
            self._recovery_timer.cancel()
            self._recovery_timer = None
        if self._current_dest:
            self.get_logger().info(
                f"Recovery: re-navigating to {self._current_dest!r}"
            )
            self._navigate_to(self._current_dest)

    def _recovery_confirm(self):
        """One-shot timer: re-post ARRIVED for a WAITING_CONFIRM recovery."""
        if self._recovery_timer:
            self._recovery_timer.cancel()
            self._recovery_timer = None
        self.get_logger().info("Recovery: re-posting ARRIVED to LCD")
        self._post_lcd_mode("ARRIVED")

    # -----------------------------------------------------------------------
    # Mapping: save waypoint + trigger SLAM save
    # -----------------------------------------------------------------------

    def _save_location_and_map(self, event_name: str):
        location_key = _SAVE_LOCATION_MAP[event_name]
        x, y, w = self._lookup_tf(location_key)
        self._write_location(location_key, x, y, w)
        self._delete_all_maps()
        self._publish("save_map")

    def _mark_location(self, event_name: str):
        location_key = _MARK_LOCATION_MAP[event_name]
        x, y, w = self._lookup_tf(location_key)
        self._write_location(location_key, x, y, w)

    def _lookup_tf(self, label: str) -> tuple[float, float, float]:
        x, y, w = 0.0, 0.0, 1.0
        if self._tf_buffer is None:
            return x, y, w
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            w = math.cos(yaw / 2.0)
            self.get_logger().info(
                f"TF lookup → {label}: x={x:.3f} y={y:.3f} w={w:.4f}"
            )
        except Exception as exc:
            self.get_logger().warn(
                f"TF lookup failed ({exc}) — saving (0,0) placeholder for {label!r}"
            )
        return x, y, w

    def _write_location(self, key: str, x: float, y: float, w: float):
        if not self._loc_yaml:
            self.get_logger().warn("locations_yaml_path not set — coordinates not saved")
            return
        if not _YAML_AVAILABLE:
            self.get_logger().warn("PyYAML not installed — coordinates not saved")
            return
        try:
            # Ensure directory exists (e.g. ~/PortaMailCapstone/config/)
            os.makedirs(os.path.dirname(self._loc_yaml), exist_ok=True)
            try:
                with open(self._loc_yaml, "r") as f:
                    data = yaml.safe_load(f) or {}
            except FileNotFoundError:
                data = {}
            if "locations" not in data:
                data["locations"] = {}
            data["locations"][key] = {
                "x": round(float(x), 4),
                "y": round(float(y), 4),
                "w": round(float(w), 4),
            }
            with open(self._loc_yaml, "w") as f:
                yaml.dump(data, f, default_flow_style=False)
            self.get_logger().info(f"Saved '{key}' to {self._loc_yaml}")
        except Exception as exc:
            self.get_logger().error(f"Failed to write locations.yaml: {exc}")

    # -----------------------------------------------------------------------
    # Map file management
    # -----------------------------------------------------------------------

    def _delete_all_maps(self):
        deleted = 0
        for pattern in [
            os.path.join(self._maps_dir, "*.yaml"),
            os.path.join(self._maps_dir, "*.pgm"),
        ]:
            for path in glob.glob(pattern):
                try:
                    os.remove(path)
                    deleted += 1
                except OSError as exc:
                    self.get_logger().warn(f"Could not delete {path}: {exc}")
        self.get_logger().info(f"Deleted {deleted} map file(s) from {self._maps_dir}")

    def _clear_saved_locations(self):
        if not self._loc_yaml or not _YAML_AVAILABLE:
            return
        try:
            with open(self._loc_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
            if data.get("locations"):
                data["locations"] = {}
                with open(self._loc_yaml, "w") as f:
                    yaml.dump(data, f, default_flow_style=False)
                self.get_logger().info("Cleared saved locations from locations.yaml")
        except Exception as exc:
            self.get_logger().warn(f"Failed to clear locations.yaml: {exc}")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _navigate_to(self, location: str):
        self._publish(location)

    def _publish(self, cmd: str):
        msg      = String()
        msg.data = cmd
        self._pub.publish(msg)
        self.get_logger().info(f"-> user_delivery_request: {cmd!r}")

    def _nav_error_human_message(self, code: str, detail: str) -> str:
        messages = {
            "GOAL_REJECTED":    "Nav2 rejected the destination. Path may be blocked.",
            "NAV2_OFFLINE":     "Navigation system is offline. Please restart.",
            "UNKNOWN_LOCATION": "Destination not found. Please re-run mapping.",
            "NAV_FAILED":       f"Could not reach destination ({detail or 'obstacle or timeout'}).",
        }
        return messages.get(code, f"Navigation error: {code}. Returning to base.")

    def _post_lcd_mode(self, mode: str):
        try:
            self._http_post("/api/mode", {"mode": mode})
            self.get_logger().info(f"-> LCD mode: {mode}")
        except Exception as exc:
            self.get_logger().warn(f"LCD mode POST error: {exc}")


def main():
    rclpy.init()
    node = LcdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
