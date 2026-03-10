#!/usr/bin/env python3
"""
lcd_bridge — ROS 2 node that bridges the PortaMail LCD Flask UI to the
navigation_coordinator node.

LCD events  →  user_delivery_request topic
system_status topic  →  POST /api/mode to LCD server

In mapping mode:
  save_location_room1/room2/origin → looks up map→base_link TF, writes
  locations.yaml, then publishes "save_map" to trigger SLAM map save.

Internal delivery state machine:
  IDLE              – waiting for a start_room event
  NAVIGATING        – Nav2 goal sent, waiting for "Status: Arrived"
  WAITING_CONFIRM   – robot at destination, waiting for delivery_confirmed event
  RETURNING         – navigating back to mailroom (dock)
"""

import glob
import json
import math
import os
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

# LCD GUI room → locations.yaml key
_ROOM_TO_LOCATION = {
    "ROOM1":  "office_101",
    "ROOM2":  "office_102",
    "ORIGIN": "mailroom",
}

# save_location edge → locations.yaml key
_SAVE_LOCATION_MAP = {
    "save_location_room1":   "office_101",
    "save_location_room2":   "office_102",
    "save_location_origin":  "mailroom",
}

# Internal bridge states
_IDLE            = "IDLE"
_NAVIGATING      = "NAVIGATING"
_WAITING_CONFIRM = "WAITING_CONFIRM"
_RETURNING       = "RETURNING"


class LcdBridge(Node):
    def __init__(self):
        super().__init__("lcd_bridge")

        self.declare_parameter("lcd_url",            "http://127.0.0.1:5050")
        self.declare_parameter("poll_hz",            2.0)
        self.declare_parameter("ros_mode",           "navigation")
        self.declare_parameter("locations_yaml_path", "")
        self.declare_parameter(
            "maps_dir",
            os.path.expanduser("~/PortaMailCapstone/maps"),
        )

        self._lcd_url      = self.get_parameter("lcd_url").get_parameter_value().string_value
        poll_hz            = self.get_parameter("poll_hz").get_parameter_value().double_value
        self._ros_mode     = self.get_parameter("ros_mode").get_parameter_value().string_value
        self._loc_yaml     = self.get_parameter("locations_yaml_path").get_parameter_value().string_value
        self._maps_dir     = self.get_parameter("maps_dir").get_parameter_value().string_value

        self._pub = self.create_publisher(String, "user_delivery_request", 10)
        self.create_subscription(String, "system_status", self._on_status, 10)

        self._bridge_state = _IDLE
        self._seen: dict[str, bool] = {}
        self._last_ts: str | None = None

        # TF2 for map→base_link lookups (used in mapping mode to save waypoints)
        if _TF2_AVAILABLE:
            self._tf_buffer   = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self.get_logger().warn("tf2_ros not available — save_location will not record TF pose")

        self._drain_history()
        self.create_timer(1.0 / poll_hz, self._poll)
        self.get_logger().info(
            f"LCD bridge ready | url={self._lcd_url} | ros_mode={self._ros_mode}"
        )

    # ------------------------------------------------------------------
    # HTTP helpers
    # ------------------------------------------------------------------

    def _http_get(self, path: str):
        url = f"{self._lcd_url}{path}"
        with urllib.request.urlopen(url, timeout=3) as resp:
            return json.loads(resp.read())

    def _http_post(self, path: str, body: dict):
        data = json.dumps(body).encode()
        req = urllib.request.Request(
            f"{self._lcd_url}{path}", data=data, method="POST"
        )
        req.add_header("Content-Type", "application/json")
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read())

    # ------------------------------------------------------------------
    # Startup: drain history so old events are not replayed
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Polling timer
    # ------------------------------------------------------------------

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
                old_keys = list(self._seen)[:200]
                for k in old_keys:
                    del self._seen[k]
            self._last_ts = evt.get("ts") or self._last_ts
            self._handle_event(evt.get("name"))

    # ------------------------------------------------------------------
    # Event dispatch
    # ------------------------------------------------------------------

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
                # Save the map without recording any TF coordinates
                self._delete_all_maps()
                self._publish("save_map")
            elif name == "go_back":
                # User left mapping without saving — discard auto-saved maps
                self._delete_all_maps()
            return

        # --- Navigation mode ---
        if name in ("start_room1", "start_room2", "start_origin") and self._bridge_state == _IDLE:
            if name == "start_origin":
                dest = "mailroom"
            else:
                active = self._get_lcd_active_room() or (
                    "ROOM1" if name == "start_room1" else "ROOM2"
                )
                dest = _ROOM_TO_LOCATION.get(active)
            if dest:
                self._navigate_to(dest)
                self._bridge_state = _NAVIGATING

        elif name == "delivery_confirmed" and self._bridge_state == _WAITING_CONFIRM:
            active = self._get_lcd_active_room()
            if active and active in _ROOM_TO_LOCATION:
                self._navigate_to(_ROOM_TO_LOCATION[active])
                self._bridge_state = _NAVIGATING
            else:
                self._navigate_to("mailroom")
                self._bridge_state = _RETURNING

    # ------------------------------------------------------------------
    # Mapping: save waypoint coordinates + trigger SLAM map save
    # ------------------------------------------------------------------

    def _save_location_and_map(self, event_name: str):
        location_key = _SAVE_LOCATION_MAP[event_name]

        # --- TF lookup ---
        x, y, w = 0.0, 0.0, 1.0   # fallback if TF unavailable
        if self._tf_buffer is not None:
            try:
                t = self._tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                w = math.cos(yaw / 2.0)   # orientation.w of 2-D quaternion
                self.get_logger().info(
                    f"TF lookup → {location_key}: x={x:.3f} y={y:.3f} w={w:.4f}"
                )
            except Exception as exc:
                self.get_logger().warn(f"TF lookup failed ({exc}) — saving (0,0) placeholder")

        # --- Write locations.yaml ---
        if self._loc_yaml and _YAML_AVAILABLE:
            try:
                with open(self._loc_yaml, "r") as f:
                    data = yaml.safe_load(f) or {}
                if "locations" not in data:
                    data["locations"] = {}
                data["locations"][location_key] = {
                    "x": round(float(x), 4),
                    "y": round(float(y), 4),
                    "w": round(float(w), 4),
                }
                with open(self._loc_yaml, "w") as f:
                    yaml.dump(data, f, default_flow_style=False)
                self.get_logger().info(
                    f"Saved '{location_key}' to {self._loc_yaml}"
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to write locations.yaml: {exc}")
        else:
            if not self._loc_yaml:
                self.get_logger().warn("locations_yaml_path not set — coordinates not saved")
            if not _YAML_AVAILABLE:
                self.get_logger().warn("PyYAML not installed — coordinates not saved")

        # --- Delete old maps, then trigger SLAM map save ---
        self._delete_all_maps()
        self._publish("save_map")

    # ------------------------------------------------------------------
    # Map file management
    # ------------------------------------------------------------------

    def _delete_all_maps(self):
        """Remove all .yaml and .pgm map files from the maps directory."""
        patterns = [
            os.path.join(self._maps_dir, "*.yaml"),
            os.path.join(self._maps_dir, "*.pgm"),
        ]
        deleted = 0
        for pattern in patterns:
            for path in glob.glob(pattern):
                try:
                    os.remove(path)
                    deleted += 1
                except OSError as exc:
                    self.get_logger().warn(f"Could not delete map file {path}: {exc}")
        self.get_logger().info(f"Deleted {deleted} map file(s) from {self._maps_dir}")

    # ------------------------------------------------------------------
    # LCD state helper
    # ------------------------------------------------------------------

    def _get_lcd_active_room(self) -> str | None:
        try:
            st = self._http_get("/api/state")
            return st.get("active_room")
        except Exception as exc:
            self.get_logger().warn(f"LCD state fetch error: {exc}")
            return None

    # ------------------------------------------------------------------
    # system_status subscriber
    # ------------------------------------------------------------------

    def _on_status(self, msg: String):
        text = msg.data
        self.get_logger().debug(f"system_status={text!r}  bridge={self._bridge_state}")

        if self._ros_mode == "navigation":
            if "Status: Arrived" in text:
                if self._bridge_state == _NAVIGATING:
                    self._post_lcd_mode("ARRIVED")
                    self._bridge_state = _WAITING_CONFIRM
                elif self._bridge_state == _RETURNING:
                    self._post_lcd_mode("DOCK_IDLE")
                    self._bridge_state = _IDLE
            elif "Error:" in text and self._bridge_state == _NAVIGATING:
                self.get_logger().warn("Nav failed — returning to dock")
                self._post_lcd_mode("DOCK_IDLE")
                self._navigate_to("mailroom")
                self._bridge_state = _RETURNING

        elif self._ros_mode == "mapping":
            if "Status: Map Saved" in text:
                self.get_logger().info("Map saved — returning to mode selection")
                # Trigger go_back via the LCD REST API so the monitoring script
                # detects MODE_SELECT and shuts down the ROS stack cleanly.
                try:
                    self._http_post("/api/edge", {"edge": "go_back"})
                except Exception as exc:
                    self.get_logger().warn(f"Could not POST go_back to LCD: {exc}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _navigate_to(self, location: str):
        self._publish(location)

    def _publish(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        self.get_logger().info(f"-> user_delivery_request: {cmd!r}")

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
