import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LcdBridgeNode(Node):
    def __init__(self):
        super().__init__('lcd_bridge')
        self.declare_parameter('lcd_base_url', 'http://127.0.0.1:5050')
        self.declare_parameter('poll_hz', 5.0)

        self._lcd_base_url = self.get_parameter('lcd_base_url').value
        self._poll_hz = float(self.get_parameter('poll_hz').value)

        self._events_pub = self.create_publisher(String, '/lcd/events', 10)
        self._state_pub = self.create_publisher(String, '/lcd/state', 10)
        self._mode_sub = self.create_subscription(
            String, '/lcd/set_mode', self._on_set_mode, 10
        )

        self._poll_timer = self.create_timer(1.0 / self._poll_hz, self._poll_loop)

        self._seen_events = []
        self._seen_limit = 200
        self._last_state_json = None

    def _poll_loop(self):
        self._poll_state()
        self._poll_events()

    def _poll_state(self):
        url = f'{self._lcd_base_url}/api/state'
        try:
            raw = self._http_get(url)
            if raw is None:
                return
            if raw != self._last_state_json:
                self._last_state_json = raw
                msg = String()
                msg.data = raw
                self._state_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to poll state: {exc}')

    def _poll_events(self):
        url = f'{self._lcd_base_url}/api/events'
        try:
            raw = self._http_get(url)
            if raw is None:
                return
            events = json.loads(raw)
            if not isinstance(events, list):
                return
            for evt in events:
                key = self._event_key(evt)
                if key is None:
                    continue
                if key in self._seen_events:
                    continue
                self._seen_events.append(key)
                if len(self._seen_events) > self._seen_limit:
                    self._seen_events = self._seen_events[-self._seen_limit :]
                msg = String()
                msg.data = json.dumps(evt)
                self._events_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to poll events: {exc}')

    def _on_set_mode(self, msg: String):
        mode = None
        data = msg.data.strip()
        if data.startswith('{'):
            try:
                payload = json.loads(data)
                mode = payload.get('mode')
            except json.JSONDecodeError:
                self.get_logger().warn('Invalid JSON on /lcd/set_mode')
                return
        else:
            mode = data

        if mode not in {'ARRIVED', 'DOCK_IDLE'}:
            self.get_logger().warn(f'Invalid mode on /lcd/set_mode: {mode}')
            return

        url = f'{self._lcd_base_url}/api/mode'
        body = json.dumps({'mode': mode}).encode('utf-8')
        req = urllib.request.Request(url, data=body, method='POST')
        req.add_header('Content-Type', 'application/json')
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                if resp.status != 200:
                    self.get_logger().warn(f'POST /api/mode failed: {resp.status}')
        except Exception as exc:
            self.get_logger().warn(f'POST /api/mode failed: {exc}')

    def _http_get(self, url):
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                if resp.status != 200:
                    self.get_logger().warn(f'GET {url} failed: {resp.status}')
                    return None
                return resp.read().decode('utf-8')
        except urllib.error.URLError as exc:
            self.get_logger().warn(f'GET {url} failed: {exc}')
            return None

    @staticmethod
    def _event_key(evt):
        if not isinstance(evt, dict):
            return None
        ts = evt.get('ts')
        name = evt.get('name') or evt.get('type')
        if ts and name:
            return f'{ts}:{name}'
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LcdBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
