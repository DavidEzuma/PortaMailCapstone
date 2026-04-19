# lcd_bridge (Phase 4.2)

This package bridges the PortaMail LCD Flask server to ROS2 using **HTTP polling** only.
It does **not** include any navigation, motor control, or mission logic.

## What Phase 4.2 does
- Polls `/api/state` and publishes JSON strings to `/lcd/state`
- Polls `/api/events` and publishes **new** events to `/lcd/events` (deduped)
- Accepts `/lcd/set_mode` messages and POSTs to `/api/mode`

## Usage
1) Run the Flask LCD server (separately).
2) Run the bridge:
```bash
ros2 run lcd_bridge lcd_bridge
```
3) View outputs:
```bash
ros2 topic echo /lcd/state
ros2 topic echo /lcd/events
```
4) Trigger LCD mode from ROS:
```bash
ros2 topic pub --once /lcd/set_mode std_msgs/msg/String "{data: 'ARRIVED'}"
ros2 topic pub --once /lcd/set_mode std_msgs/msg/String "{data: 'DOCK_IDLE'}"
```

## Parameters
- `lcd_base_url` (default: `http://127.0.0.1:5050`)
- `poll_hz` (default: `5.0`)
