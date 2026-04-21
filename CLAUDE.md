# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PortaMail is an autonomous indoor mail-delivery robot for hospitals/corporate campuses. It carries mail in a 3D-printed dual-slot bucket, navigates a pre-mapped single floor using SLAM + Nav2, and is operated via a 7" touchscreen. Built as a Texas A&M ESET capstone project (ESET 419 Fall 2025 / ESET 420 Spring 2026).

**Team roles relevant to this repo:**
- David Ezuma — Software (Raspberry Pi, ROS 2, SLAM, GUI)
- Henry Lovelace — Software
- Sebastian Mejias — Electrical Engineer & Project Manager (PCB design, `capstone_PCB/`)

## Confirmed Hardware (from Purchase Orders + PCB Schematic)

> **MCU NOTE**: The custom PCB (`capstone_PCB/`) uses an **ESP32-DevKitC** as the low-level MCU, not the Teensy 4.0. The Teensy was the original plan; the ESP32 is what is actually on the PCB. The `firmware/esp32_driver/esp32_driver.ino` is the active firmware. The Teensy driver (`firmware/teensy_driver/`) is legacy/backup only.

### Compute
| Component | Part | Notes |
|---|---|---|
| Raspberry Pi 5 (8GB) | — | Ubuntu 24.04 + ROS 2 Jazzy |
| ESP32-DevKitC | PCB U6 | Primary low-level MCU; micro-ROS over USB serial (CP2102 → `/dev/ttyUSB0` when LiDAR not connected, `/dev/ttyUSB1` when LiDAR is on USB0) |
| Teensy 4.0 w/ headers | DigiKey 1568-16997-ND | **Legacy/backup only** — not on the custom PCB |
| 64GB microSDXC | DigiKey 6318-SDCS3/64GB-ND | Boot media for Pi |
| RPi Camera Module 3 | DigiKey 2648-SC1223-ND | Side-mounted |

### Sensors
| Component | Part | Interface |
|---|---|---|
| SLAMTEC RPLIDAR A2M12 | Amazon B0G2XZXJQ3 | USB CP2102 adapter → `/dev/ttyUSB0`, **256000 baud** |
| Adafruit BNO055 IMU | DigiKey 1528-1426-ND | I2C → ESP32 GPIO 21 (SDA) / GPIO 22 (SCL); PCB U2; I2C addr 0x28 |
| HC-SR04 Ultrasonic | PCB connector TBD | TRIG/ECHO GPIOs unconfirmed — verify physical board (**ECHO is 5V output — voltage divider required**) |

### Drivetrain
| Component | Part | Notes |
|---|---|---|
| DFRobot FIT0186 gearmotor | DigiKey 1738-1106-ND (×2) | 12V, 251 RPM no-load, 90:1 gear ratio |
| Encoder | Built into FIT0186 | Hall-effect, 8 PPR on motor shaft |
| Motor driver IC | VNH5019ATR-E (×2) | PCB U3 (left), U4 (right); 5.5–24V, 5A/phase; INA/INB/EN interface (NOT L298N) |
| Wheels | Amazon B0CTJWQ7G7 | 1/10 RC Monster Truck 2.8" (71.1mm diameter, 35.56mm radius) |
| Caster | — | 50mm swivel caster (front) |

### Power
| Component | Notes |
|---|---|
| TalentCell 12V Li-ion 122Wh | Amazon B016BJCRUO; XT60 output; runtime target ≥2 hrs (cutoff: 11.0V) |
| DFRobot DC-DC buck converter | PCB U5 (DFR1202); 12V → 5V, 2A continuous; powers ESP32, IMU, ultrasonic |
| Thermal fuse | PCB F1; irreversible — must replace physically if blown |
| P-channel MOSFET switch | PCB Q1 (IRF4905PBF); firmware-controlled main power gate |
| Rocker switch (×2) | DigiKey EG4777-ND; main power cutoff |

### Display
| Component | Notes |
|---|---|
| Waveshare 7" TFT RGB touchscreen | DigiKey 2648-SC1227-ND; connects to Raspberry Pi (DSI or HDMI) |

### Mechanical
- **Chassis**: DFRobot 2WD aluminum platform
- **Drive config**: Differential drive — 2 powered rear wheels + 1 front swivel caster
- **3D-printed parts (PETG)**: Mail bucket (16"×16"×3", dual-slot), motor casings, LiDAR mount arm, caster spacer
- **Bumpers (TPU)**: C-channel profile, "PortaMail" embossed, ~23" height
- **Full assembly**: `src/portamail_navigator/meshes/` (STL files), STEP assembly in Drive

## PCB Schematic — Verified Electrical Design

Source: `capstone_PCB/capstone_PCB.kicad_sch` (authoritative). Do not re-analyze the schematic for pin mappings — use the tables below.

### Power Distribution

```
12V Battery (J3 barrel jack)
    → F1 thermal fuse (irreversible)
    → Q1 IRF4905PBF P-channel MOSFET (firmware-controlled gate)
        ├─→ J5, J6 fan connectors @ 12V
        └─→ U5 DFR1202 buck converter → +5V (VCC rail, 2A cont.)
                ├─→ ESP32 (U6)
                ├─→ BNO055 IMU (U2)
                ├─→ HC-SR04 ultrasonic (J4)
                └─→ U3, U4 VNH5019 VCC pins (logic) AND VBAT pins (motor supply)
```

| Rail | Nominal | Source |
|---|---|---|
| +12V | 11–13.5V | Battery direct |
| +5V (VCC) | 4.8–5.2V | U5 DFR1202 |
| +3.3V | 3.25–3.35V | ESP32 internal LDO |

> **CONFIRMED PCB BUG — VBAT on 5V rail**: PCB netlist analysis confirmed that U3/U4 VNH5019 VBAT pins (pad 12, motor supply) are wired to the VCC net (5V), not to the 12V battery rail. The VNH5019 requires minimum 5.5V on VBAT; at 5V motor output is severely limited or nonexistent. **Rework required**: cut trace on pad 12 from VCC pour, bodge wire to 12V via (do NOT unsolder IC — cut trace + topside wire is safer).

### ESP32 GPIO Assignments (verified from `capstone_PCB.kicad_pcb` net assignments)

#### Motor Drivers (PCB-verified)

| GPIO | Net (PCB) | Connected To |
|---|---|---|
| 5  | Net-(U3-INA)       | U3 VNH5019 INA — left motor direction A |
| 25 | Net-(U3-INB)       | U3 VNH5019 INB — left motor direction B |
| 26 | Net-(U3-ENA/DIAGA) | U3 VNH5019 ENA — left motor PWM speed |
| 27 | Net-(U3-ENB/DIAGB) | U3 VNH5019 ENB — left motor half-bridge B enable (drive HIGH) |
| 33 | Net-(U4-INA)       | U4 VNH5019 INA — right motor direction A |
| 13 | Net-(U4-INB)       | U4 VNH5019 INB — right motor direction B |
| 14 | Net-(U4-ENA/DIAGA) | U4 VNH5019 ENA — right motor PWM speed |
| 4  | Net-(U4-ENB/DIAGB) | U4 VNH5019 ENB — right motor half-bridge B enable (drive HIGH) |

#### IMU (PCB-verified)

| GPIO | Net (PCB) | Connected To |
|---|---|---|
| 21 | Net-(U2-SDA) | BNO055 SDA — R6 5.1kΩ pull-up to +5V |
| 22 | Net-(U2-SCL) | BNO055 SCL — R7 5.1kΩ pull-up to +5V |

#### Encoders (PCB connectors P3/P4 — left/right assignment TBD)

| GPIO | Connector | Notes |
|---|---|---|
| 19 | P3 pin 1 | Encoder channel — interrupt capable |
| 18 | P3 pin 2 | Encoder channel |
| 17 | P4 pin 1 | Encoder channel — interrupt capable |
| 16 | P4 pin 2 | Encoder channel |

#### VNH5019 PWM / CS_DIS (PCB-verified)

PWM (pin 7) and CS_DIS (pin 6) on U3 and U4 are shorted together at a common wire junction, then that junction connects to VCC (5V = HIGH). This is intentional:
- **PWM=HIGH**: chip enabled; speed controlled entirely via ENA/DIAGA PWM. Correct for the firmware's approach.
- **CS_DIS=HIGH**: current sense output disabled. CS pins (pad 8) are unconnected on both U3/U4 — current sensing is not used.

#### Ultrasonic / Other

| GPIO | Connected To | Notes |
|---|---|---|
| 2  | LED → R5 100Ω | Status LED |
| 32 | VCC (5V rail) | Connected to 5V power plane — **not usable as GPIO** |
| 23 | VCC (5V rail) | Connected to 5V power plane — **not usable as GPIO** |

> **HC-SR04 TRIG/ECHO**: GPIO5 is INA_MD1 (left motor), not TRIG. Ultrasonic pin assignments are not traced in the PCB file and must be verified on the physical board.

### VNH5019ATR-E Motor Driver Interface (U3 = left, U4 = right)

The VNH5019 is **not** an L298N. The control interface differs:

| INA | INB | Result |
|---|---|---|
| 0 | 0 | Coast (free wheel) |
| 1 | 0 | Forward |
| 0 | 1 | Reverse |
| 1 | 1 | Brake (shorts motor windings) |

Speed is controlled by PWM on **ENA/DIAG** pin. ENB pins (GPIO 27 left, GPIO 4 right) are driven HIGH in `setup()` to keep both half-bridges active. EN/DIAG is bidirectional — goes LOW on fault (overcurrent, thermal shutdown).

### Connectors

| Ref | Type | Pinout |
|---|---|---|
| J1 | 4-pin terminal | Encoder signals: Pin1=A, Pin2=+5V, Pin3=B, Pin4=+5V (no GND pin — which motor TBD) |
| J2 | 4-pin terminal | Left motor power + encoder power: Pin1=OUTA/motor+, Pin2=OUTB/motor−, Pin3=GND, Pin4=+5V |
| J3 | Barrel jack | +12V in (center), GND (barrel) |
| J4 | 2-pin terminal | Right motor power: Pin1=OUTA/motor+, Pin2=OUTB/motor− |
| J5, J6 | JST XH 3-pin | Fan: +12V, GND, PWM |
| J7 | USB-C | Data / optional 5V power |

### Critical Hardware Warnings

1. **VBAT on 5V (PCB bug)** — VNH5019 motor supply is 5V not 12V; needs rework (trace cut + bodge to 12V rail).
2. **HC-SR04 ECHO is 5V** — ESP32 GPIO max is 3.3V. Voltage divider required on ECHO line.
3. **I2C pull-ups (R6, R7) go to +5V** — SDA/SCL swing to 5V; verify ESP32 pins 21/22 are 5V-tolerant.
4. **F1 is irreversible** — thermal fuse must be physically replaced if blown.
5. **VNH5019 EN/DIAG is bidirectional** — use a series resistor when driving; monitor for fault (pin goes LOW).
6. **Encoder Hall sensors powered at +5V** — outputs will be 5V logic into 3.3V ESP32 GPIO. Confirm and add level shifting if needed.

## Build Commands

All commands run on the Raspberry Pi from the repo root, after sourcing ROS 2:

```bash
source /opt/ros/jazzy/setup.bash

# Build all packages
colcon build --packages-select portamail_coordinator portamail_navigator lcd_bridge

# Source workspace after build
source install/setup.bash
```

First-time Pi setup (installs ROS 2 Jazzy, dependencies, Docker for micro-ROS agent):
```bash
./setup_and_build.sh
```

### LCD Flask Server Tests

Run from `LCD_web/portamail_ui/` with the venv active:

```bash
cd LCD_web/portamail_ui
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt

# All tests
python -m pytest tests/

# Single test file
python -m pytest tests/test_api_contract.py

# Single test
python -m pytest tests/test_api_contract.py::ApiContractTests::test_get_state_shape_is_stable
```

Smoke test (requires running server):
```bash
bash LCD_web/portamail_ui/tools/smoke_test.sh
python3 LCD_web/portamail_ui/tools/smoke_regression.py
```

## Running the System

### Primary Startup (physical robot, all-in-one)
```bash
./launch_portamail.sh
```
Starts the LCD Flask server, opens Chromium in kiosk mode, waits for mode selection on the touchscreen, launches the appropriate ROS 2 stack, and monitors the Back button to restart the mode loop. This is the normal operating mode on the Pi.

### Convenience Scripts (physical robot, manual)
```bash
./launch_mapping.sh       # hardware stack + SLAM Toolbox
./launch_hardware.sh      # hardware stack only
./launch_simulation.sh    # Gazebo Classic headless simulation
```

### Manual Launch (multi-terminal)

**Simulation (desktop, no hardware)**
```bash
# Terminal 1
ros2 launch portamail_navigator mapping.launch.py use_mock_driver:=true use_real_lidar:=false

# Terminal 2
ros2 launch portamail_coordinator bringup.launch.py mode:=mapping
```

**Physical Robot — Mapping**
```bash
# Terminal 1
ros2 launch portamail_navigator mapping.launch.py use_mock_driver:=false use_real_lidar:=true

# Terminal 2
ros2 launch portamail_coordinator bringup.launch.py mode:=mapping
```

**Physical Robot — Navigation**
```bash
# navigation.launch.py auto-selects the newest portamail_map_*.yaml from ~/PortaMailCapstone/maps/
# Staged startup: t=0 hardware, t+3 AMCL+map_server, t+8 Nav2, t+10 coordinator+lcd_bridge
ros2 launch portamail_navigator navigation.launch.py

# Or via coordinator bringup:
ros2 launch portamail_coordinator bringup.launch.py mode:=navigation
```

### micro-ROS Agent
`hardware.launch.py` starts the micro-ROS agent automatically. To run manually:
```bash
./run_agent.sh              # default: /dev/ttyUSB0
./run_agent.sh /dev/ttyUSB1 # when LiDAR is on ttyUSB0
```
- **amd64 (laptop)**: uses `microros/micro-ros-agent:jazzy` Docker image
- **arm64 (Pi)**: uses native agent built from source in `~/microros_ws`

**Foxglove visualization**: `ws://<robot-ip>:8765`

**Map autosave**: maps saved every 30 s to `~/PortaMailCapstone/maps/portamail_map_YYYYMMDD_HHMMSS.{yaml,pgm}`

**LiDAR diagnostic** (no ROS needed):
```bash
python3 test_lidar.py
```

**LCD Flask server** (standalone):
```bash
cd LCD_web/portamail_ui
source venv/bin/activate
python app.py           # serves at http://127.0.0.1:5050
LCD_SHOW_DEBUG_PANEL=1 python app.py   # enable debug overlay
```

## Architecture

### Package: `portamail_navigator`

Low-level hardware abstraction, sensor stack, and SLAM.

- **`firmware/esp32_driver/esp32_driver.ino`** — Active firmware for ESP32-WROOM-32. Publishes `/cmd_vel` subscriber, `/wheel/odom` (20 Hz), `/imu/data` (20 Hz, BNO055 via GPIO 21/22 I2C), `/ultrasonic/range` (2 Hz). Uses LEDC PWM (`ledcAttach`/`ledcWrite`), `IRAM_ATTR` ISRs, `portDISABLE_INTERRUPTS` for atomic encoder reads. Reconnect state machine pings agent every 1 s without rebooting. Safety timeout 500 ms. Current tuned: `SLEW_RATE=3.0` m/s², `PWM_MIN=70`, BEST_EFFORT QoS on `/cmd_vel`.

- **`firmware/teensy_driver/teensy_driver.ino`** — Legacy/backup. Not on custom PCB. Teensy-specific: `analogWrite`, `Wire` on pins 18/19, native USB → `/dev/ttyACM0`.

- **`src/mock_driver.cpp`** — Simulated diff-drive node. Subscribes `/cmd_vel`, integrates at 20 Hz, publishes `odom` + `odom→base_link` TF. Use on desktop.

- **`src/map_autosave_node.cpp`** — Calls `/slam_toolbox/save_map` periodically, writes timestamped files. Params: `output_directory`, `autosave_interval_sec` (default 30), `save_on_startup`.

- **`launch/hardware.launch.py`** — Brings up robot_state_publisher, micro-ROS agent (native), RPLIDAR A2M12 (256000 baud), EKF. Args: `use_lidar`, `use_mcu`, `mcu_port`, `use_imu` (default false), `use_ekf`.

- **`launch/mapping.launch.py`** — Includes hardware.launch.py + SLAM Toolbox (lifecycle, auto configure→activate with 2 s delay) + map_autosave_node + joystick + Foxglove bridge.

- **`launch/navigation.launch.py`** — Full navigation stack with staged startup: hardware at t=0, AMCL+map_server at t+3, Nav2 at t+8, coordinator+lcd_bridge at t+10. **Auto-selects newest `portamail_map_*.yaml`** from `~/PortaMailCapstone/maps/`. Aborts if no map found.

- **`config/ekf.yaml`** / **`config/ekf_no_imu.yaml`** — `robot_localization` EKF configs. `ekf_no_imu.yaml` is the default (odometry only) until BNO055 is wired. Pass `use_imu:=true` to switch.

- **`config/slam.yaml`** — SLAM Toolbox: 5 cm resolution, Ceres solver, loop closure enabled, `minimum_time_interval: 0.1`, `transform_publish_period: 0.05`.

- **`config/portamail_nav2_params.yaml`** — Nav2 parameters for navigation mode (controller, planner, AMCL, BT navigator).

- **`urdf/portamail.urdf`** — Robot description. Sensor frame offsets (`laser_joint`, `imu_joint`, `ultrasonic_joint`) are **estimates** and must be measured on the physical robot before SLAM.

### Package: `portamail_coordinator`

High-level mission logic and LCD bridge.

- **`src/navigation_coordinator.cpp`** — Single ROS 2 node. Two modes (`start_mode` param):
  - **MAPPING**: accepts `save_map` command → calls `/slam_toolbox/save_map`.
  - **NAVIGATION**: maps named location strings from `locations.yaml` → sends Nav2 `NavigateToPose` action goals.
  - Subscribes `user_delivery_request` (`std_msgs/String`), publishes `system_status` (`std_msgs/String`).

- **`scripts/lcd_bridge.py`** — Primary bridge node. Polls LCD Flask server at 2 Hz. Delivery state machine:

  ```
  IDLE → NAVIGATING → WAITING_CONFIRM → RETURNING → IDLE
  ```

  Multi-room queue: `start_room1`/`start_room2` events build an ordered queue (e.g. `["office_101", "office_102", "mailroom"]`). Mailroom is always last and **skips WAITING_CONFIRM** (robot docks without human confirmation). State persists atomically to `~/.portamail_delivery_state.json` on every transition (crash recovery). In mapping mode: handles TF lookup → writes `locations.yaml`, manages map files.

- **`config/locations.yaml`** — Named waypoints in `map` frame. Currently: `mailroom`, `office_101`, `office_102`, `lobby`.

- **`launch/bringup.launch.py`** — Launches coordinator + lcd_bridge. Args: `mode` (`mapping`|`navigation`), `lcd_url`.

### Package: `lcd_bridge`

Standalone ament_python package (separate from `portamail_coordinator`). Publishes raw LCD events/state as ROS topics for debugging. Publishes `/lcd/events` (JSON), `/lcd/state` (JSON). Subscribes `/lcd/set_mode` (`"ARRIVED"` or `"DOCK_IDLE"`).

### LCD Flask Server (`LCD_web/portamail_ui/`)

Flask + Socket.IO app served at `http://127.0.0.1:5050`.

**Internal layer structure:**
- `app.py` — Flask app entry point; `LCD_SHOW_DEBUG_PANEL` env var controls debug overlay
- `state/model.py` — Shared mutable state dict (in-memory singleton)
- `state/state_machine.py` — Screen transitions, room queuing, delivery log (`logs/delivery_log.txt`)
- `interface/contract.py` — Canonical sets of valid MODES, SCREENS, BIT_KEYS, EDGE_EVENTS
- `interface/api_handlers.py` — REST API routes
- `interface/validators.py` — Input validation against contract
- `interface/event_store.py` — Timestamped event log for polling
- `transport/socketio_handlers.py` — Socket.IO push events to browser

**Screens:** `MODE_SELECT`, `HOME`, `ARRIVED`, `CONFIRM_SELECT`, `CONFIRM_ACK`, `DELIVERING_ROOM1`, `DELIVERING_ROOM2`, `MAPPING`, `SAVE_MAP_SELECT`, `SAVE_LOCATION_SELECT`, `PROCESSING`, `NAV_ERROR`

**Modes:** `DOCK_IDLE`, `ARRIVED`, `MAPPING`

**Key REST API:**
| Endpoint | Method | Purpose |
|---|---|---|
| `/api/state` | GET | Full state: mode, screen, selected_room, pending_rooms, active_room, bits, events |
| `/api/events[?since_ts=...]` | GET | Timestamped event log |
| `/api/mode` | POST `{"mode": "..."}` | Drive UI transitions |
| `/api/edge` | POST `{"edge": "..."}` | Trigger named state-machine edge |

Socket.IO: server emits `state_update` on every state change.

**Tools:**
- `tools/external_sim.py` — Simulates navigator (polls events, sends ARRIVED after delay)
- `tools/smoke_regression.py` — Automated regression against running server

### Key ROS Topics / Actions
| Topic / Action | Type | Source → Sink |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / joystick → ESP32 (micro-ROS) |
| `/wheel/odom` | `nav_msgs/Odometry` | ESP32 encoders → EKF |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 BNO055 → EKF |
| `/ultrasonic/range` | `sensor_msgs/Range` | ESP32 HC-SR04 → Nav2 costmap |
| `/scan` | `sensor_msgs/LaserScan` | RPLIDAR A2M12 → SLAM / AMCL |
| `/odom` | `nav_msgs/Odometry` | Mock driver → Nav2 (simulation only) |
| `user_delivery_request` | `std_msgs/String` | lcd_bridge.py → coordinator |
| `system_status` | `std_msgs/String` | coordinator → lcd_bridge.py |
| `/navigate_to_pose` | Nav2 action | coordinator → Nav2 |
| `/slam_toolbox/save_map` | service | coordinator / map_autosave_node → SLAM |

### TF Frame Tree
`map` → `odom` → `base_link` → `laser`

EKF (real hardware) or `mock_driver` (simulation) publishes `odom→base_link`. SLAM Toolbox (mapping) or AMCL (navigation) publishes `map→odom`.

## Critical Hardware Constants (Firmware)

| Constant | Current Value | Basis | Notes |
|---|---|---|---|
| `WHEEL_RADIUS` | `0.03556` m | 2.8" wheels ÷ 2 | Verify on actual tire |
| `WHEEL_BASE` | `0.20` m | Estimated | **Measure actual track width** |
| `TICKS_PER_REV` | `720.0` | 8 PPR × 90:1 = 720 (single-edge) | **Calibrate physically** |
| `MAX_SPEED_MPS` | `0.89` | 2 MPH cap | Hard cap in firmware |
| `PWM_MIN` | `70` | Motor dead zone | Tune per motor |
| `SLEW_RATE` | `3.0` m/s² | Tuned | Ramp rate 0→full speed |

### Joystick Tuning (`launch/joystick.launch.py`)

| Parameter | Value | Notes |
|---|---|---|
| `scale_linear.x` | `0.447` | 1 MPH operational max |
| `scale_angular.yaw` | `1.0` | Turn rate scale |
| `deadzone` | `0.05` | 5% stick dead zone |
| `autorepeat_rate` | `20` Hz | Held-input refresh |
| `enable_button` | `5` | Right bumper (xpadneo) — must hold |

## Physical Calibration Checklist

All items below must be done on the assembled physical robot before SLAM or autonomous navigation will work correctly.

### 1. VBAT PCB Rework (BLOCKING)

**Problem**: U3/U4 VNH5019 VBAT (pad 12) tied to VCC (5V), below the 5.5V minimum.

**Fix**: On each IC (U3 and U4):
1. Locate the trace connecting pad 12 to the 5V copper pour
2. Score and cut the trace with an X-Acto knife; verify cut with multimeter
3. Solder 30 AWG wire: pad 12 → nearest 12V via (TP26, C3 positive leg, or J3 input side)
4. Insulate with Kapton tape

Do NOT unsolder the IC — the PowerSSO-30 package has fine-pitch pins and an exposed thermal pad; desoldering risks lifted pads.

### 2. Verify Ultrasonic ECHO Pin

Confirm HC-SR04 ECHO wire goes to the correct ESP32 GPIO. Add voltage divider (e.g. 10kΩ/20kΩ) on ECHO line — HC-SR04 outputs 5V, ESP32 GPIO max is 3.3V.

### 3. Wire and Enable BNO055 IMU

BNO055 is purchased but not yet wired. When ready:
1. Connect per PCB (GPIO 21 SDA, GPIO 22 SCL); I2C address 0x28
2. Verify with `ros2 topic echo /imu/data`
3. Launch with `use_imu:=true`

### 4. Measure Track Width (WHEEL_BASE)

Measure centre-to-centre distance between drive wheels. Update `WHEEL_BASE` in `esp32_driver.ino`.

### 5. Calibrate Ticks Per Revolution (TICKS_PER_REV)

Theoretical: 8 PPR × 90:1 = 720. Physical procedure:
1. Drive exactly 1.000 m in a straight line; record tick count `N`
2. `TICKS_PER_REV = N × WHEEL_RADIUS × 2π / distance_m`
3. Verify: odometry error < 2 cm over 1 m

### 6. Tune PWM Dead Zone (PWM_MIN)

Send small `cmd_vel` and increase PWM until both wheels move. Both motors should start at the same value; mismatch causes pulling from rest. Typical range: 50–100 out of 255.

### 7. Update URDF Sensor Frame Offsets

**File**: `urdf/portamail.urdf` — three fixed joints at bottom of file.

| Frame | Joint | Current (estimate) | Measure |
|---|---|---|---|
| RPLIDAR A2M12 | `laser_joint` | `xyz="0.1 0.0 0.3"` | X = forward from axle; Z = scan height |
| BNO055 IMU | `imu_joint` | `xyz="0.0 0.0 0.05"` | IMU chip position on PCB |
| Ultrasonic | `ultrasonic_joint` | `xyz="0.2 0.0 0.05"` | Front edge of chassis |

> **SLAM will not work correctly with wrong `laser_joint` offsets.** Measure carefully.

### 8. EKF Covariance Tuning

After physical calibration, verify EKF with a known square path. Adjust covariance values in `config/ekf.yaml` if drift is excessive. Starting odometry covariances are set in firmware: `pose[0]=0.01`, `pose[7]=0.01`, `pose[35]=0.10`.

## Project Specifications

- **Delivery destinations**: 1 home (mailroom) + 2 offices; autonomous round-trip required
- **Payload**: ≤15 lbs in dual-slot 3D-printed bucket
- **Goal tolerance**: distance < 0.2 m AND velocity < 0.05 m/s
- **Nav latency target**: < 50 ms user input → motor response
- **Battery runtime**: ≥ 2 hours (test: run until pack drops to 11.0V)
- **E-stop**: cuts motor power within 0.5 s of activation
- **Maps saved to**: `~/PortaMailCapstone/maps/`
