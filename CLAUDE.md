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
| ESP32-DevKitC | PCB U6 | Primary low-level MCU; micro-ROS over USB serial (CP2102 → `/dev/ttyUSB1`) |
| Teensy 4.0 w/ headers | DigiKey 1568-16997-ND | **Legacy/backup only** — not on the custom PCB |
| 64GB microSDXC | DigiKey 6318-SDCS3/64GB-ND | Boot media for Pi |
| RPi Camera Module 3 | DigiKey 2648-SC1223-ND | Side-mounted |

### Sensors
| Component | Part | Interface |
|---|---|---|
| SLAMTEC RPLIDAR A2M12 | Amazon B0G2XZXJQ3 | USB CP2102 adapter → `/dev/ttyUSB0`, **256000 baud** |
| Adafruit BNO055 IMU | DigiKey 1528-1426-ND | I2C → ESP32 GPIO 21 (SDA) / GPIO 22 (SCL); PCB U2; I2C addr 0x28 |
| HC-SR04 Ultrasonic | PCB J4 | TRIG → ESP32 GPIO 5; ECHO → ESP32 GPIO 18 (**5V output — voltage divider required**) |

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
        ├─→ U3, U4 VNH5019 motor drivers @ 12V
        ├─→ J5, J6 fan connectors @ 12V
        └─→ U5 DFR1202 buck converter → +5V (2A cont.)
                ├─→ ESP32 (U6)
                ├─→ BNO055 IMU (U2)
                └─→ HC-SR04 ultrasonic (J4)
```

| Rail | Nominal | Source |
|---|---|---|
| +12V | 11–13.5V | Battery direct |
| +5V | 4.8–5.2V | U5 DFR1202 |
| +3.3V | 3.25–3.35V | ESP32 internal LDO |

### ESP32 GPIO Assignments (from schematic net names)

| GPIO | Net | Connected To |
|---|---|---|
| 4 | ENC_R_B | Right encoder Hall B (J2) |
| 5 | TRIG | HC-SR04 trigger (J4 pin 3) |
| 13 | INA_MD1 | U3 VNH5019 INA — left motor direction A |
| 14 | INB_MD1 | U3 VNH5019 INB — left motor direction B |
| 16 | EN_MD1 | U3 VNH5019 EN/DIAG — left motor PWM speed |
| 17 | EN_MD2 | U4 VNH5019 EN/DIAG — right motor PWM speed |
| 18 | ECHO | HC-SR04 echo (J4 pin 4) — **5V output, voltage divider required** |
| 19 | ENB_MD1 | U3 VNH5019 ENB — left motor (see VNH5019 note) |
| 21 | SDA_IMU | BNO055 SDA — R6 5.1kΩ pull-up to +5V |
| 22 | SCL_IMU | BNO055 SCL — R7 5.1kΩ pull-up to +5V |
| 23 | ENB_MD2 | U4 VNH5019 ENB — right motor |
| 25 | ENC_R_A | Right encoder Hall A (J2, interrupt) |
| 26 | INB_MD2 | U4 VNH5019 INB — right motor direction B |
| 27 | INA_MD2 | U4 VNH5019 INA — right motor direction A |
| 32 | ENC_L_A | Left encoder Hall A (J1, interrupt) |
| 33 | ENC_L_B | Left encoder Hall B (J1) |
| 2 | LED | Status LED → R5 100Ω series |

### VNH5019ATR-E Motor Driver Interface (U3 = left, U4 = right)

The VNH5019 is **not** an L298N. The control interface differs:

| INA | INB | Result |
|---|---|---|
| 0 | 0 | Coast (free wheel) |
| 1 | 0 | Forward |
| 0 | 1 | Reverse |
| 1 | 1 | Brake (shorts motor windings) |

Speed is controlled by PWM on **EN/DIAG** pin. EN/DIAG is bidirectional — it also goes LOW on fault (overcurrent, thermal shutdown). The VNH5019 has built-in thermal shutdown, UVLO, and cross-conduction prevention.

> **FIRMWARE NOTE**: `esp32_driver.ino` is currently written for L298N (ENA/ENB/IN1-IN4 interface). It must be updated to use INA/INB/EN per the VNH5019 interface above before it will work with the custom PCB.

### Connectors

| Ref | Type | Pinout |
|---|---|---|
| J1 | 4-pin terminal | Left encoder: A, B, +5V, GND |
| J2 | 4-pin terminal | Right encoder: A, B, +5V, GND |
| J3 | Barrel jack | +12V in (center), GND (barrel) |
| J4 | 4-pin terminal | Ultrasonic: GND, +5V, TRIG, ECHO |
| J5, J6 | JST XH 3-pin | Fan: +12V, GND, PWM |
| J7 | USB-C | Data / optional 5V power |

### Passives

| Ref | Value | Purpose |
|---|---|---|
| R1, R2, R3 | 150Ω | Motor driver sense/enable circuits |
| R4 | 100kΩ | Pull-up |
| R5 | 100Ω | LED series current limiter |
| R6, R7 | 5.1kΩ | I2C SDA/SCL pull-ups to +5V |
| C3 | 1000µF | 12V rail bulk decoupling |
| C4 | 0.1µF | Local IC bypass |

### Test Points

| TP | Signal | Expected |
|---|---|---|
| TP8 | SDA_IMU | 0–3.3V I2C pulses |
| TP9 | SCL_IMU | 0–3.3V I2C clock |
| TP16 | LED | 0–3.3V |
| TP17–TP20 | Motor enable signals | 0–3.3V PWM |
| TP21–TP24 | Motor output phases | 0 or 12V switching |
| TP25 | +3.3V | 3.25–3.35V |
| TP26 | +12V | 11.0–13.5V |
| TP27 | GND | 0V reference |
| TP28 | +5V | 4.8–5.2V |

### Critical Hardware Warnings (from schematic)

1. **HC-SR04 ECHO is 5V** — ESP32 GPIO max is 3.3V. Voltage divider (e.g. 10kΩ/20kΩ) or level shifter required on the ECHO line.
2. **I2C pull-ups (R6, R7) go to +5V** — SDA/SCL swing to 5V. Verify ESP32 pins 21/22 are 5V-tolerant on the specific module used.
3. **F1 is irreversible** — thermal fuse must be physically replaced if it blows; it is not resettable.
4. **VNH5019 EN/DIAG is bidirectional** — use a series resistor when driving; monitor for fault (pin goes LOW).
5. **Encoder Hall sensors powered at +5V (J1/J2)** — outputs will be 5V logic into 3.3V ESP32 GPIO. Confirm encoder output voltage on actual FIT0186 and add level shifting if needed.

## Build Commands

All commands run on the Raspberry Pi from the repo root, after sourcing ROS 2:

```bash
source /opt/ros/jazzy/setup.bash

# Build all packages (includes lcd_bridge ROS 2 package)
colcon build --packages-select portamail_coordinator portamail_navigator lcd_bridge

# Source workspace after build
source install/setup.bash
```

First-time Pi setup (installs ROS 2 Jazzy, dependencies, Docker for micro-ROS agent):
```bash
./setup_and_build.sh
```

## Running the System

### Primary Startup (physical robot, all-in-one)
```bash
./launch_portamail.sh
```
This script handles the full startup sequence: starts the LCD Flask server, opens Chromium in kiosk mode on the touchscreen, waits for the user to select Mapping or Navigation on screen, launches the appropriate ROS 2 stack, and monitors for the Back button to restart the mode loop. This is the intended normal operating mode on the Pi.

### Convenience Scripts (physical robot, manual)
```bash
./launch_mapping.sh       # hardware stack + SLAM Toolbox (no LCD server)
./launch_hardware.sh      # hardware stack only (no SLAM, no coordinator)
./launch_simulation.sh    # Gazebo Classic headless simulation
```

### Manual Launch (multi-terminal)

**Simulation (desktop, no hardware)**
```bash
# Terminal 1: Navigator with mock driver + SLAM
ros2 launch portamail_navigator mapping.launch.py use_mock_driver:=true use_real_lidar:=false

# Terminal 2: Coordinator in mapping mode
ros2 launch portamail_coordinator bringup.launch.py mode:=mapping
```

**Physical Robot (Raspberry Pi)**
```bash
# Terminal 1: Mapping (hardware stack + SLAM Toolbox + map autosave + Foxglove)
ros2 launch portamail_navigator mapping.launch.py use_mock_driver:=false use_real_lidar:=true

# Terminal 2: Coordinator
ros2 launch portamail_coordinator bringup.launch.py mode:=mapping
ros2 launch portamail_coordinator bringup.launch.py mode:=navigation
```

### micro-ROS Agent
`hardware.launch.py` starts the micro-ROS agent automatically via Docker. To run it manually:
```bash
./run_agent.sh              # Teensy: /dev/ttyACM0 (default)
./run_agent.sh /dev/ttyUSB1 # ESP32: /dev/ttyUSB1 (when LiDAR is on USB0)
```
Note: `run_agent.sh` uses the `humble` Docker image (no Jazzy arm64 image exists); this is an intentional workaround.

**Foxglove visualization**: `ws://<robot-ip>:8765` (started automatically by `mapping.launch.py`)

**Map autosave**: maps saved every 30 s to `~/PortaMailCapstone/maps/portamail_map_YYYYMMDD_HHMMSS.{yaml,pgm}`. Override with launch args `map_output_dir`, `autosave_interval_sec`, `autosave_enabled`.

**If multiple USB serial devices are connected** (LiDAR + MCU), confirm the LiDAR enumerates as `/dev/ttyUSB0`. Use the stable by-id path if needed:
```bash
ls /dev/serial/by-id/   # look for usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_*
```

**LiDAR diagnostic** (quick connectivity check, no ROS needed):
```bash
python3 test_lidar.py   # runs from repo root; prints 5 scan summaries
```

## Architecture

### Package: `portamail_navigator`

Low-level hardware abstraction, sensor stack, and SLAM.

- **`firmware/teensy_driver/teensy_driver.ino`** — Micro-ROS sketch for Teensy 4.0. Subscribes to `/cmd_vel`; publishes `/wheel/odom` (20 Hz), `/imu/data` (20 Hz, BNO055 via I2C pins 18/19), and `/ultrasonic/range` (10 Hz). Motor driver controlled via INA/INB+PWM. FIT0186 encoders on interrupt pins. 1-second safety timeout stops motors if no `cmd_vel` arrives. Requires Arduino libraries: `Adafruit BNO055`, `Adafruit Unified Sensor`. Appears as `/dev/ttyACM0` on the Pi (native USB CDC).

- **`firmware/teensy_driver/locale_stubs.c`** — Stub implementations of locale functions missing from the Teensyduino toolchain. Required for micro_ros_arduino to link correctly; see the `platform.local.txt` fix in the Memory file.

- **`firmware/esp32_driver/esp32_driver.ino`** — Drop-in alternative firmware for ESP32-WROOM-32 (if the Teensy is unavailable). Publishes the same ROS topics so the Pi-side stack needs no changes. Key differences from the Teensy driver: uses **L298N** motor driver (ENA/ENB/IN1–IN4), LEDC PWM (`ledcAttach`/`ledcWrite`, ESP32 Arduino core v3.x), `IRAM_ATTR` ISRs, `portDISABLE_INTERRUPTS` for atomic sections, I2C initialized as `Wire.begin(21, 22)`. Appears as `/dev/ttyUSB1` (on-board CP2102 bridge) when LiDAR is on USB0. Flash at 921600 baud; micro-ROS runs at 115200. Avoid GPIO 6–11 (internal flash) and GPIO 12 (boot strap).

- **`src/mock_driver.cpp`** — Simulated differential-drive node (`mock_teensy_driver`). Subscribes to `cmd_vel`, integrates kinematics at 20 Hz, publishes `odom` + the `odom→base_link` TF. Use on desktop in place of real hardware.

- **`src/map_autosave_node.cpp`** — Periodically calls `/slam_toolbox/save_map` and writes timestamped `.yaml`/`.pgm` map files. Header at `include/portamail_navigator/map_autosave_node.hpp`. Config in `config/map_autosave.yaml`. Parameters: `output_directory`, `filename_prefix` (default `portamail_map`), `autosave_interval_sec` (default 30), `save_on_startup` (default false).

- **`config/ekf.yaml`** — `robot_localization` EKF fusing `/wheel/odom` + `/imu/data` (BNO055). Use when IMU is physically connected.

- **`config/ekf_no_imu.yaml`** — EKF with wheel odometry only. Default until BNO055 is wired.

- **`config/slam.yaml`** — SLAM Toolbox parameters (5 cm resolution, Ceres solver, loop closure enabled, `use_sim_time: false`, `minimum_time_interval: 0.1`, `transform_publish_period: 0.05`).

- **`config/map_autosave.yaml`** — Default parameters for `map_autosave_node`.

- **`launch/hardware.launch.py`** — Brings up robot_state_publisher (from URDF), Micro-ROS agent (Teensy bridge), RPLIDAR A2M12 (`/dev/ttyUSB0`, 256000 baud), and EKF. The `use_imu` argument (default `false`) selects which EKF config is loaded. There is **no Pi-side IMU node** — the BNO055 is wired to the Teensy via I2C; the Teensy firmware publishes `/imu/data` over micro-ROS when the IMU is connected.

- **`launch/mapping.launch.py`** — Includes `hardware.launch.py`, optionally starts `mock_driver`, starts **SLAM Toolbox** as a lifecycle node (auto configure→activate with 2 s delay), starts `map_autosave_node`, joystick, and Foxglove bridge.

- **`urdf/portamail.urdf`** — Robot description with full TF tree. Includes `laser`, `imu_link`, and `ultrasonic_link` fixed frames attached to `base_link`. **Sensor frame offsets are estimates and must be measured on the physical robot before SLAM is run.**

- **`urdf/portamail_classic.urdf`** — Legacy URDF from an earlier design; not currently used.

- **`launch/gazebo_classic_mapping.launch.py`** — Launches Gazebo Classic headless simulation with SLAM. Used by `launch_simulation.sh`. Separate from the mock_driver simulation path.

- **`src/robot_state_publisher.cpp`** — Not compiled (not in CMakeLists). Dead code; sensor TF frames are now in the URDF.

### Package: `portamail_coordinator`

High-level mission logic and LCD bridge.

- **`src/navigation_coordinator.cpp`** — Single ROS 2 node (`navigation_coordinator`). Two modes (set by `start_mode` parameter):
  - **MAPPING**: Accepts only `save_map` command → calls `/slam_toolbox/save_map` service.
  - **NAVIGATION**: Accepts named location strings from `locations.yaml` → sends Nav2 `NavigateToPose` action goals.
  - Subscribes to `user_delivery_request` (`std_msgs/String`), publishes to `system_status` (`std_msgs/String`).

- **`config/locations.yaml`** — Named waypoints in `map` frame (x, y, orientation.w). Edit to add delivery destinations. Currently: `mailroom`, `office_101`, `office_102`, `lobby`.

- **`launch/bringup.launch.py`** — Launches coordinator + `lcd_bridge` node. Arguments: `mode` (`mapping` | `navigation`), `lcd_url` (default `http://127.0.0.1:5050`).

- **`scripts/lcd_bridge.py`** — The actual `lcd_bridge` node launched by `bringup.launch.py` (entry point for `portamail_coordinator`). Polls the LCD Flask server (2 Hz). Internal delivery state machine: `IDLE → NAVIGATING → WAITING_CONFIRM → RETURNING`. In navigation mode: maps `start_room1`/`start_room2`/`start_origin` events → `user_delivery_request`; posts `ARRIVED`/`DOCK_IDLE` to the LCD. In mapping mode: handles `save_location_room1/room2/origin` (TF lookup → write `locations.yaml` + trigger map save), `mark_location_*` (write coordinates only), `save_map_now`/`go_back` (delete old maps ± clear locations). Parameters: `lcd_url`, `poll_hz`, `ros_mode`, `locations_yaml_path`, `maps_dir`.

### Package: `lcd_bridge`

Standalone ROS 2 ament_python package (separate from `portamail_coordinator`). A simpler bridge that publishes raw LCD events/state as ROS topics — useful for debugging or alternative coordinator integrations.

- Publishes `/lcd/events` (`std_msgs/String`, JSON) — new events from `/api/events`, deduplicated
- Publishes `/lcd/state` (`std_msgs/String`, JSON) — LCD state on change from `/api/state`
- Subscribes `/lcd/set_mode` (`std_msgs/String`) — accepts `"ARRIVED"` or `"DOCK_IDLE"` (plain string or JSON `{"mode": "..."}`) and POSTs to `/api/mode`
- Parameters: `lcd_base_url` (default `http://127.0.0.1:5050`), `poll_hz` (default 5.0)

### LCD Flask Server (`LCD_web/portamail_ui/`)

The touchscreen UI is a Flask web app served at `http://127.0.0.1:5050`. Displayed full-screen in Chromium kiosk mode by `launch_portamail.sh`. The Python venv and dependencies are managed automatically by the startup script.

Key REST API endpoints consumed by `lcd_bridge`/`lcd_bridge.py`:
| Endpoint | Method | Purpose |
|---|---|---|
| `/api/events[?since_ts=...]` | GET | Timestamped UI event log; optional filter by timestamp |
| `/api/state` | GET | Current screen name + active room |
| `/api/mode` | POST `{"mode": "..."}` | Drive UI transitions (`ARRIVED`, `DOCK_IDLE`) |
| `/api/edge` | POST `{"edge": "..."}` | Trigger named state-machine edge (e.g. `map_saved`) |

### Diagnostics & Utilities

- **`test_lidar.py`** (repo root) — Quick LiDAR connectivity check using `adafruit_rplidar`. Connects to `/dev/ttyUSB0` at 256000 baud, prints 5 scan summaries, then disconnects. No ROS required. Run as `python3 test_lidar.py`.

- **`external_controller/controller.py`** — Standalone delivery simulator for testing the LCD Flask API without a running ROS stack. Polls `/api/events`, simulates navigation by waiting 3 s then POSTing `ARRIVED`, and returns to `DOCK_IDLE` on `delivery_confirmed`. Config in `external_controller/config.json` (`lcd_base_url`, `poll_interval_sec`).

- **`scripts/splashscreen.py`** — Boot splash rendered to `/dev/fb0` (framebuffer) before the X/Wayland session starts. Shows maroon "Welcome to PortaMail" gradient with animated loading bar. Exits when the X11 socket appears at `/tmp/.X11-unix/X0` or after 90 s. Requires Pillow (`PIL`); falls back gracefully if unavailable. Started by `scripts/apply_splash.sh`.

### Key ROS Topics / Actions
| Topic / Action | Type | Source → Sink |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / joystick → Teensy (micro-ROS USB) |
| `/wheel/odom` | `nav_msgs/Odometry` | Teensy encoders → EKF (micro-ROS USB) |
| `/imu/data` | `sensor_msgs/Imu` | Teensy BNO055 I2C → EKF (micro-ROS USB) |
| `/ultrasonic/range` | `sensor_msgs/Range` | Teensy HC-SR04 → Nav2 costmap (micro-ROS USB) |
| `/scan` | `sensor_msgs/LaserScan` | RPLIDAR A2M12 (`/dev/ttyUSB0`) → SLAM Toolbox |
| `/odom` | `nav_msgs/Odometry` | Mock driver → Nav2 (simulation only) |
| `user_delivery_request` | `std_msgs/String` | lcd_bridge.py → coordinator |
| `system_status` | `std_msgs/String` | coordinator → lcd_bridge.py |
| `/lcd/events` | `std_msgs/String` | lcd_bridge pkg → (debug consumers) |
| `/lcd/state` | `std_msgs/String` | lcd_bridge pkg → (debug consumers) |
| `/lcd/set_mode` | `std_msgs/String` | (external) → lcd_bridge pkg → Flask |
| `/navigate_to_pose` | Nav2 action | coordinator → Nav2 |
| `/slam_toolbox/save_map` | service | coordinator / map_autosave_node → SLAM Toolbox |

### TF Frame Tree
`map` → `odom` → `base_link` → `laser`

The EKF (real hardware) or `mock_driver` (simulation) publishes the `odom→base_link` transform. SLAM Toolbox (mapping mode) or AMCL (navigation mode) publishes `map→odom`.

## Critical Hardware Constants (Firmware)

Constants in `firmware/teensy_driver/teensy_driver.ino` (and mirrored identically in `firmware/esp32_driver/esp32_driver.ino`) that must be calibrated on the physical robot:

| Constant | Current Value | Basis | Notes |
|---|---|---|---|
| `WHEEL_RADIUS` | `0.03556` m | 2.8" wheels ÷ 2 | Verify on actual tire |
| `WHEEL_BASE` | `0.20` m | Estimated | **Measure the actual track width** |
| `TICKS_PER_REV` | `720.0` | 8 PPR × 90:1 gear = 720 (single-edge) | **Calibrate physically**: drive exactly 1 m, count ticks |
| `MAX_SPEED_MPS` | `0.89` | 2 MPH limit | Matches SMART Goal #11 |
| `PWM_MIN` | `70` | Motor dead zone | Tune per motor |

**EKF sensor selection**: Pass `use_imu:=false` (default) for odometry-only fusion. Once BNO055 is physically wired to the Teensy's I2C pins and the Teensy firmware publishes `/imu/data`, pass `use_imu:=true` to load the full fusion config.

## Physical Calibration Checklist

All items below must be done on the assembled physical robot before SLAM or autonomous navigation will work correctly. Current values in firmware are estimates from datasheets and purchase orders.

---

### 1. Verify Ultrasonic ECHO Pin

**File**: `firmware/teensy_driver/teensy_driver.ino` — `#define ULTRASONIC_ECHO 26`

Check the final PCB schematic to confirm HC-SR04 Echo wire goes to Teensy pin 26. Update `ULTRASONIC_ECHO` if different. Also confirm Trig is on pin 27.

> **Important**: HC-SR04 Echo outputs **5V**. Teensy 4.0 GPIO is 3.3V-tolerant only. A voltage divider or level shifter is required on the Echo line.

---

### 2. Wire and Enable BNO055 IMU

**Current state**: BNO055 is purchased but not yet soldered/wired.

Steps when ready:
1. Connect BNO055 SDA → Teensy pin 18, SCL → Teensy pin 19 (Wire defaults)
2. I2C address is 0x28 (ADR pin floating/low)
3. Flash the firmware — `imu_ready = bno.begin()` will return `true`; confirm on serial monitor or by echoing `/imu/data`:
   ```bash
   ros2 topic echo /imu/data
   ```
4. Launch with EKF fusion enabled:
   ```bash
   ros2 launch portamail_navigator hardware.launch.py use_imu:=true
   ```

---

### 3. Measure Track Width (WHEEL_BASE)

**File**: `firmware/teensy_driver/teensy_driver.ino` — `const float WHEEL_BASE = 0.20f;`

Measure centre-to-centre distance between the two drive wheels on the assembled chassis. Update `WHEEL_BASE` with the actual measurement in metres.

---

### 4. Calibrate Ticks Per Revolution (TICKS_PER_REV)

**File**: `firmware/teensy_driver/teensy_driver.ino` — `const float TICKS_PER_REV = 720.0f;`

**Theoretical value**: FIT0186 = 8 PPR on motor shaft × 90:1 gear ratio = **720 ticks/output revolution** (single rising-edge on A only).

**Physical calibration procedure**:
1. Mark the wheel at a reference point on the floor.
2. Add a temporary `Serial.println(left_ticks)` (or echo `/wheel/odom`) to read tick count.
3. Drive the robot in a straight line for **exactly 1.000 m** (tape measure).
4. Record the tick count `N`.
5. Calculate: `TICKS_PER_REV = N / (1.0 / (2π × 0.03556))` = `N × 0.22348`
   - Or equivalently: `TICKS_PER_REV = N × WHEEL_RADIUS × 2π / distance_m`
6. Update the constant and re-flash.
7. Re-run the 1 m test to verify odometry error is < 2 cm.

---

### 5. Verify Wheel Radius (WHEEL_RADIUS)

**File**: `firmware/teensy_driver/teensy_driver.ino` — `const float WHEEL_RADIUS = 0.03556f;`

**From PO**: 2.8" RC Monster Truck wheels → radius = 35.56 mm = 0.03556 m. But loaded radius (robot weight on tire) may differ slightly. After performing the 1 m calibration above, if odometry is still off, measure the actual outer diameter of the inflated/mounted tire and update accordingly.

---

### 6. Tune PWM Dead Zone (PWM_MIN)

**File**: `firmware/teensy_driver/teensy_driver.ino` — `const int PWM_MIN = 70;`

`PWM_MIN` is the minimum PWM value that overcomes motor stiction and produces actual motion. Tune per motor:

1. Send small `cmd_vel` commands and increase PWM until both wheels start moving smoothly.
2. Check that both left and right motors start at the same `PWM_MIN` (if not, the robot will pull to one side from rest).
3. Typical range for 12V gearmotors: 50–100 out of 255.

---

### 7. Update URDF Sensor Frame Offsets

**File**: `urdf/portamail.urdf` — three fixed joints at the bottom of the file.

All three sensor origin translations are **estimates** and must be replaced with measurements from the assembled robot. Measure in metres from `base_link` origin (centre of the drive axle at floor level):

| Frame | Joint name | Current (estimate) | Measure |
|---|---|---|---|
| RPLIDAR A2M12 | `laser_joint` | `xyz="0.1 0.0 0.3"` | X = forward offset of LiDAR centre from axle; Z = height of LiDAR scan plane |
| BNO055 IMU | `imu_joint` | `xyz="0.0 0.0 0.05"` | Actual position of IMU chip on PCB relative to chassis origin |
| Ultrasonic ranger | `ultrasonic_joint` | `xyz="0.2 0.0 0.05"` | Front edge of chassis; confirm mounting height |

> **SLAM will not work correctly with wrong `laser_joint` offsets.** The LiDAR Z height and forward/lateral position directly affect scan matching and map quality. Measure these carefully.

---

### 8. EKF Covariance Tuning

**Files**: `config/ekf.yaml`, `config/ekf_no_imu.yaml`

After physical calibration, verify EKF behaviour by running the robot in a known square path and checking `/odom` vs ground truth. If drift is excessive:
- Reduce `odom0` covariance values (trust encoders more) if odometry is well-calibrated
- Reduce `imu0` covariance for angular velocity if BNO055 gyro is stable

Current odometry covariances are set in firmware (`odom_msg.pose.covariance[0]=0.01`, `[7]=0.01`, `[35]=0.10`). These are starting estimates only.

---

## Project Specifications

- **Delivery destinations**: 1 home (mailroom) + 2 offices; autonomous round-trip required
- **Payload**: ≤15 lbs in dual-slot 3D-printed bucket
- **Goal tolerance**: distance < 0.2 m AND velocity < 0.05 m/s
- **Nav latency target**: < 50 ms user input → motor response
- **Battery runtime**: ≥ 2 hours (test: run until pack drops to 11.0V)
- **E-stop**: cuts motor power within 0.5 s of activation
- **Maps saved to**: `~/PortaMailCapstone/maps/` (auto-saved by `map_autosave_node`; also configurable via `map_save_path` parameter on coordinator)
