# Research: Full-Stack Architecture (2026-03-30)

## Problem Statement

Understand the complete PortaMail system end-to-end — boot sequence, ROS node
graph, hardware boundary, LCD UI ↔ coordinator data flow — to diagnose
existing problems and plan the Nav2 integration + autonomous delivery features.

## Requirements

- Cover the full path from power-on → mode select → ROS stack → hardware
- Identify all ROS nodes, topics, TF frames, and service/action interfaces
- Document the LCD state machine and how events flow into ROS
- Identify architectural gaps that block Nav2 / delivery completion
- Scope: read-only research; no changes during this session

---

## Findings

### Relevant Files

| File | Purpose | Key Lines |
|---|---|---|
| `launch_portamail.sh` | Top-level startup; Flask → Chromium → mode select → ROS | 66–373 |
| `LCD_web/portamail_ui/app.py` | Flask server, REST API, SocketIO push | all |
| `LCD_web/portamail_ui/state/state_machine.py` | Screen state machine (transitions, guards) | all |
| `src/portamail_coordinator/scripts/lcd_bridge.py` | ROS ↔ LCD bridge; delivery state machine | all |
| `src/portamail_coordinator/src/navigation_coordinator.cpp` | Nav2 goal dispatch, mapping save | all |
| `src/portamail_coordinator/config/locations.yaml` | Named waypoints in `map` frame | all |
| `src/portamail_navigator/launch/mapping.launch.py` | Hardware + SLAM + joystick + Foxglove | all |
| `src/portamail_navigator/launch/hardware.launch.py` | micro-ROS agent, LiDAR, EKF, RSP | all |
| `src/portamail_navigator/launch/joystick.launch.py` | joy_node + teleop_twist_joy | all |
| `src/portamail_navigator/firmware/esp32_driver/esp32_driver.ino` | ESP32 micro-ROS firmware | all |
| `src/portamail_navigator/config/slam.yaml` | SLAM Toolbox parameters | all |
| `src/portamail_navigator/config/ekf.yaml` | robot_localization EKF (odom + IMU) | all |
| `src/portamail_navigator/urdf/portamail.urdf` | Robot description + TF tree | all |
| `src/portamail_coordinator/launch/bringup.launch.py` | Coordinator + lcd_bridge launch | all |

---

### Boot / Launch Flow

```
Power on → Ubuntu 24.04 auto-login
  → scripts/splashscreen.py  (framebuffer maroon splash, waits for X11)
  → launch_portamail.sh
       1. pkill stale app.py / lcd_bridge
       2. pip install + start LCD Flask server (port 5050, PID saved)
       3. Wait ≤60 s for Wayland socket OR X11 socket
       4. Launch Chromium kiosk → http://127.0.0.1:5050
       5. Poll /api/events (since_ts anchor) for select_mapping | select_navigation
       6a. MAPPING  → ros2 launch mapping.launch.py (MAP_PID)
                    → ros2 launch bringup.launch.py mode:=mapping (COORD_PID)
       6b. NAVIGATION → ros2 launch bringup.launch.py mode:=navigation (COORD_PID)
       7. Monitor /api/state every 2 s; on MODE_SELECT transition → _kill_ros() → loop
```

`_kill_ros()` sequence:
1. SIGINT → `sllidar_node` (motor stop handler)
2. SIGINT → MAP_PID + COORD_PID (propagates to ros2 launch children)
3. Sleep 5 s (clean shutdown window)
4. SIGKILL survivors by name: `micro_ros_agent`, `sllidar_node`, `lcd_bridge`,
   `sync_slam_toolbox_node`, `foxglove_bridge`, `mock_driver`, `map_autosave_node`,
   `navigation_coordinator`, `robot_state_publisher`, `joy_node`, `teleop_node`,
   `ekf_node`
5. SIGKILL MAP_PID + COORD_PID; wait; clear PIDs

---

### ROS Node Graph

#### Mapping Mode

```
/micro_ros_agent          USB serial (/dev/ttyUSB1) ↔ ESP32
/esp32 (micro-ROS)
    publishes:  /wheel/odom   [nav_msgs/Odometry, 20 Hz]
                /imu/data     [sensor_msgs/Imu, 20 Hz, when BNO055 wired]
                /ultrasonic/range [sensor_msgs/Range, 2 Hz]
    subscribes: /cmd_vel      [geometry_msgs/Twist, BEST_EFFORT depth=1]

/rplidar_node              /dev/ttyUSB0, 256000 baud
    publishes: /scan          [sensor_msgs/LaserScan]

/robot_state_publisher    (portamail.urdf)
    publishes: /tf_static     (base_link → laser, imu_link, ultrasonic_link)
               /robot_description

/ekf_node                  (robot_localization, ekf_no_imu.yaml default)
    subscribes: /wheel/odom
    publishes:  /odometry/filtered  [nav_msgs/Odometry]
               /tf  (odom → base_link)

/slam_toolbox              (lifecycle: configure → active, 2 s delay)
    subscribes: /scan, /tf, /tf_static
    publishes:  /map          [nav_msgs/OccupancyGrid]
               /tf  (map → odom)
    service:   /slam_toolbox/save_map

/map_autosave_node         saves ~/PortaMailCapstone/maps/ every 30 s

/joy_node                  /dev/input/js0
/teleop_node               joy → cmd_vel (RB enable, left stick)

/foxglove_bridge           ws://0.0.0.0:8765

/navigation_coordinator    (mapping mode)
    subscribes: user_delivery_request
    calls:      /slam_toolbox/save_map (on "save_map" command)

/lcd_bridge                (mapping mode)
    polls:      http://127.0.0.1:5050/api/events (2 Hz)
    handles:    save_location_room1/room2/origin → TF lookup → locations.yaml
                save_map_now / go_back
    publishes:  user_delivery_request
```

#### Navigation Mode

```
(hardware stack NOT launched — no LiDAR, no SLAM, no EKF)

/navigation_coordinator    (navigation mode)
    subscribes: user_delivery_request
    calls:      /navigate_to_pose  [nav2_msgs/NavigateToPose action]
    publishes:  system_status

/lcd_bridge                (navigation mode)
    polls:      /api/events
    handles:    start_room1 → publish "office_101"
                start_room2 → publish "office_102"
                delivery_confirmed → publish next room or "mailroom"
    monitors:   system_status "Status: Arrived" → POST ARRIVED/DOCK_IDLE to LCD
```

---

### TF Frame Tree

```
map
 └─ odom                    (SLAM Toolbox in mapping / AMCL in navigation)
     └─ base_link            (EKF / mock_driver)
         ├─ laser             (xyz="0.1 0.0 0.3" — ESTIMATE, needs measurement)
         ├─ imu_link          (xyz="0.0 0.0 0.05" — ESTIMATE)
         └─ ultrasonic_link   (xyz="0.2 0.0 0.05" — ESTIMATE)
```

All three sensor offsets in `urdf/portamail.urdf` are **uncalibrated estimates**.
SLAM quality is directly affected by the `laser` frame offset.

---

### LCD State Machine

Screens (from `state_machine.py`):

```
MODE_SELECT
  ─[select_mapping]──→  MAPPING
  ─[select_navigation]─→ HOME

MAPPING
  ─[back]──────────────→ MODE_SELECT

HOME (navigation entry)
  ─[start_room1]───────→ PROCESSING
  ─[start_room2]───────→ PROCESSING
  ─[back]──────────────→ MODE_SELECT

PROCESSING
  ─[arrived]───────────→ ARRIVED

ARRIVED
  ─[delivery_confirmed]→ PROCESSING (if queued) | HOME (if done)
  ─[back]──────────────→ HOME
```

REST API used by `lcd_bridge.py`:

| Endpoint | Method | Purpose |
|---|---|---|
| `/api/events?since_ts=...` | GET | Timestamped event log with filter |
| `/api/state` | GET | Current screen + active room |
| `/api/mode` | POST `{"mode":"..."}` | Drive UI: ARRIVED, DOCK_IDLE |
| `/api/edge` | POST `{"edge":"..."}` | Trigger edge: map_saved, etc. |

---

### Hardware Boundary (ESP32 ↔ Pi)

```
ESP32 (micro-ROS, USB serial 115200)
  Flash port: /dev/ttyUSB0 (921600 baud, Arduino IDE)
  Runtime:    /dev/ttyUSB1 (when LiDAR on USB0); /dev/ttyUSB0 (alone)

  Reconnect state machine:
    WAITING_AGENT → ping_agent(500ms, 3) → AGENT_AVAILABLE
    AGENT_AVAILABLE → create_entities → AGENT_CONNECTED
    AGENT_CONNECTED → ping_agent every loop → (fail) → AGENT_DISCONNECTED
    AGENT_DISCONNECTED → fini_entities → WAITING_AGENT

  Motor control (VNH5019):
    Left:  INA=13, INB=14, ENA=16, ENB=19
    Right: INA=27, INB=26, ENA=17, ENB=23
    Speed: LEDC PWM on ENA/ENA pins, 5000 Hz, 8-bit

  Encoders (interrupt-driven):
    Left:  A=32, B=33
    Right: A=25, B=4

  Sensors:
    HC-SR04: TRIG=5, ECHO=18 (5V — voltage divider required)
    BNO055:  SDA=21, SCL=22 (Wire.begin(21,22))
    LED:     GPIO 2
```

---

### Architectural Gaps

These are the key missing pieces blocking autonomous delivery:

1. **Nav2 not in codebase** — `navigation_coordinator.cpp` calls
   `/navigate_to_pose` but Nav2 is never launched. Neither `bringup.launch.py`
   nor `launch_portamail.sh` starts `nav2_bringup`. Navigation mode will block
   waiting for the action server indefinitely.

2. **No map loading at navigation startup** — navigation mode skips the hardware
   stack entirely. There is no `map_server` node started to load the saved
   `.yaml`/`.pgm` map for AMCL or Nav2.

3. **AMCL missing** — No localization node in navigation mode. Without AMCL (or
   similar), the `map→odom` transform is never published and Nav2 cannot plan.

4. **No hardware stack in navigation mode** — `cmd_vel` from Nav2 will never
   reach the ESP32 because the micro-ROS agent + hardware nodes are not launched.

5. **`locations.yaml` waypoints untested** — Coordinates are placeholders
   (`mailroom: {x:0, y:0, w:1.0}`). These must be saved via the mapping UI
   (`save_location_room1/room2/origin` events) before navigation can work.

6. **URDF sensor frames uncalibrated** — All three sensor offsets are estimates.
   `laser_joint` offset directly degrades SLAM quality and loop closure.

7. **No delivery queue persistence** — `lcd_bridge.py` holds a two-room queue
   in memory. If the coordinator crashes mid-delivery the queue is lost.

8. **`system_status` topic parsing fragile** — `lcd_bridge.py` checks
   `"Status: Arrived"` as a substring. Any change to coordinator's status
   string format silently breaks the ARRIVED trigger.

9. **No obstacle avoidance configuration** — Nav2 costmap plugins not configured.
   The ultrasonic sensor publishes `/ultrasonic/range` but it is not wired into
   any costmap layer.

10. **BNO055 not yet wired** — EKF runs odometry-only (`ekf_no_imu.yaml`).
    Yaw drift during navigation will accumulate; especially noticeable in wide
    open spaces with sparse LiDAR features.

---

### Dependencies

**External ROS packages (required, must be installed):**
- `slam_toolbox` — mapping
- `robot_localization` — EKF
- `sllidar_ros2` — LiDAR driver
- `foxglove_bridge` — visualization
- `joy` — joystick
- `teleop_twist_joy` — joystick twist
- `nav2_bringup`, `nav2_*` — **not yet launched, needed for navigation**
- `micro_ros_agent` — built from source in `~/microros_ws`

**Python (LCD server venv):**
- Flask, Flask-SocketIO, eventlet (managed by `requirements.txt`)

---

### External Research

Not conducted — all findings derived from codebase.

---

### Technical Constraints

- **Pi 5, arm64, Ubuntu 24.04, ROS 2 Jazzy** — no apt/snap packages for
  micro-ROS agent on Jazzy arm64; must stay on source build in `~/microros_ws`.
- **SLAM Toolbox is a lifecycle node** — must configure→activate before it
  processes scans. The 2-second timer delay is load-dependent; on a busy Pi
  under SLAM it may need extending.
- **Fast-DDS shared memory** — `/dev/shm/fastrtps_*` files must be cleaned
  between sessions (done by `launch_portamail.sh`). If not cleaned, all nodes
  fall back to UDP, increasing message latency.
- **USB enumeration order** — LiDAR enumerates as `/dev/ttyUSB0`; ESP32 as
  `/dev/ttyUSB1` when both connected. Use stable by-id paths to avoid
  misdetection across reboots.
- **VNH5019 ENB pins** must be driven HIGH in `setup()` — if ENB floats,
  the motor half-bridge is disabled and no current flows regardless of
  INA/INB/ENA state.

---

## Open Questions

1. Which map file should navigation mode load? Should the user select from a
   list, or should it auto-load the newest map in `~/PortaMailCapstone/maps/`?

2. Should Nav2 be launched as part of `launch_portamail.sh` in navigation mode,
   or is a separate terminal / launch file preferred?

3. Are delivery locations being saved via the LCD mapping UI (`save_location_*`
   events), or will `locations.yaml` be edited manually?

4. Should the ultrasonic sensor be added to the Nav2 costmap as an obstacle
   layer, or is LiDAR-only acceptable for the capstone demo?

5. When is the BNO055 expected to be physically wired? IMU fusion significantly
   improves localization in long corridors.

---

## Recommendations

### Immediate (unblock navigation mode)

1. **Add Nav2 launch to navigation mode** — Create `navigation.launch.py` that
   starts `map_server` (load latest map), `amcl`, `nav2_bringup`, the hardware
   stack (micro-ROS agent + LiDAR + EKF), and the coordinator. Wire it into
   `launch_portamail.sh` alongside the existing mapping path.

2. **Populate `locations.yaml`** — Run a full mapping session, use the LCD
   `save_location_*` events to write real coordinates, then verify by
   re-reading the file.

3. **Calibrate URDF `laser_joint`** — Measure the LiDAR's forward/lateral
   offset from the drive axle centre and its scan-plane height. Update
   `urdf/portamail.urdf`. Re-run SLAM to validate map quality.

### Near-term (improve reliability)

4. **Add series resistors to motor GPIO lines** — 100 Ω on each of the 8
   control lines (INA, INB, ENA, ENB × 2 motors) to protect ESP32 outputs
   from VNH5019 switching noise.

5. **Calibrate WHEEL_BASE and TICKS_PER_REV** — Follow the procedure in
   `CLAUDE.md` sections 3 and 4. Current values are datasheet estimates.

6. **Wire BNO055 to ESP32** — I2C (GPIO 21/22). Enables full EKF fusion.
   Use `use_imu:=true` in hardware launch after verification.

### Planning (capstone demo readiness)

7. **Nav2 parameter tuning** — Configure inflation radius, costmap resolution,
   and planner tolerances for the hospital-corridor environment.

8. **Delivery queue persistence** — Write in-progress delivery state to a file
   so the coordinator can recover after a crash or mode-switch.

9. **E-stop integration** — Verify that the e-stop cuts motor power within 0.5 s
   per spec. The current firmware 500 ms safety timeout covers software-level
   stops; confirm the PCB hardware path as well.
