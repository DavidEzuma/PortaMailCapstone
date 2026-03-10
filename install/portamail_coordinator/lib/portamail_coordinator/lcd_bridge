#!/usr/bin/env python3
"""
lcd_bridge — ROS 2 node that bridges the PortaMail LCD Flask UI to the
navigation_coordinator node.

LCD events  →  user_delivery_request topic
system_status topic  →  POST /api/mode to LCD server

Internal delivery state machine:
  IDLE              – waiting for a start_room event
  NAVIGATING        – Nav2 goal sent, waiting for "Status: Arrived"
  WAITING_CONFIRM   – robot at destination, waiting for delivery_confirmed event
  RETURNING         – navigating back to mailroom (dock)
"""

import json
import urllib.error
import urllib.parse
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# LCD GUI room → locations.yaml key
_ROOM_TO_LOCATION = {
    "ROOM1": "office_101",
    "ROOM2": "office_102",
}

# Internal bridge states
_IDLE = "IDLE"
_NAVIGATING = "NAVIGATING"
_WAITING_CONFIRM = "WAITING_CONFIRM"
_RETURNING = "RETURNING"


class LcdBridge(Node):
    def __init__(self):
        super().__init__("lcd_bridge")

        self.declare_parameter("lcd_url", "http://127.0.0.1:5050")
        self.declare_parameter("poll_hz", 2.0)
        # 'navigation' or 'mapping' — matches the coordinator's start_mode
        self.declare_parameter("ros_mode", "navigation")

        self._lcd_url = self.get_parameter("lcd_url").get_parameter_value().string_value
        poll_hz = self.get_parameter("poll_hz").get_parameter_value().double_value
        self._ros_mode = self.get_parameter("ros_mode").get_parameter_value().string_value

        self._pub = self.create_publisher(String, "user_delivery_request", 10)
        self.create_subscription(String, "system_status", self._on_status, 10)

        self._bridge_state = _IDLE
        self._seen: dict[str, bool] = {}  # "ts|name" dedup cache
        self._last_ts: str | None = None

        # Drain historical events on startup so we don't replay stale actions.
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
            # Keep dedup cache bounded
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

        # --- Mapping mode: only forward save_map ---
        if self._ros_mode == "mapping":
            if name == "save_map":
                self._publish("save_map")
            return

        # --- Navigation mode ---
        if name in ("start_room1", "start_room2") and self._bridge_state == _IDLE:
            # Fetch active_room from LCD state to honour queue ordering
            active = self._get_lcd_active_room() or (
                "ROOM1" if name == "start_room1" else "ROOM2"
            )
            dest = _ROOM_TO_LOCATION.get(active)
            if dest:
                self._navigate_to(dest)
                self._bridge_state = _NAVIGATING

        elif name == "delivery_confirmed" and self._bridge_state == _WAITING_CONFIRM:
            # Check whether the GUI has queued another room
            active = self._get_lcd_active_room()
            if active and active in _ROOM_TO_LOCATION:
                self._navigate_to(_ROOM_TO_LOCATION[active])
                self._bridge_state = _NAVIGATING
            else:
                # All deliveries done — return to dock
                self._navigate_to("mailroom")
                self._bridge_state = _RETURNING

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
                self.get_logger().info("Map saved successfully")
                # Stay in MAPPING mode — do not post DOCK_IDLE

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
