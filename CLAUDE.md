# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PortaMail is an autonomous indoor mail-delivery robot for hospitals/corporate campuses. It carries mail in a 3D-printed dual-slot bucket, navigates a pre-mapped single floor using SLAM + Nav2, and is operated via a 7" touchscreen. Built as a Texas A&M ESET capstone project (ESET 419 Fall 2025 / ESET 420 Spring 2026).

**Team roles relevant to this repo:**
- David Ezuma — Software (Raspberry Pi, ROS 2, SLAM, GUI)
- Henry Lovelace — Firmware (Teensy / low-level motor control)

## Confirmed Hardware (from Purchase Orders)

### Compute
| Component | Part | Notes |
|---|---|---|
| Raspberry Pi 5 (8GB) | — | Ubuntu 24.04 + ROS 2 Jazzy |
| Teensy 4.0 w/ headers | DigiKey 1568-16997-ND | Low-level MCU; micro-ROS over USB serial |
| 64GB microSDXC | DigiKey 6318-SDCS3/64GB-ND | Boot media for Pi |
| RPi Camera Module 3 | DigiKey 2648-SC1223-ND | Side-mounted |

### Sensors
| Component | Part | Interface |
|---|---|---|
| SLAMTEC RPLIDAR A2M12 | Amazon B0G2XZXJQ3 | UART → Pi GPIO14/15 = `/dev/serial0`, 115200 baud |
| Adafruit BNO055 IMU | DigiKey 1528-1426-ND | I2C → Teensy SDA/SCL (**not yet wired**; Teensy publishes `/imu/data` over micro-ROS) |

### Drivetrain
| Component | Part | Notes |
|---|---|---|
| DFRobot FIT0186 gearmotor | DigiKey 1738-1106-ND (×2) | 12V, 251 RPM no-load, 90:1 gear ratio |
| Encoder | Built into FIT0186 | Hall-effect, 8 PPR on motor shaft |
| Motor driver IC | DigiKey 497-13073-1-ND (×2) | STMicro, MultiPowerSO-30, 5.5–24V; INA/INB+PWM interface |
| Wheels | Amazon B0CTJWQ7G7 | 1/10 RC Monster Truck 2.8" (71.1mm diameter, 35.56mm radius) |
| Caster | — | 50mm swivel caster (front) |

### Power
| Component | Notes |
|---|---|
| TalentCell 12V Li-ion 122Wh | Amazon B016BJCRUO; XT60 output; runtime target ≥2 hrs (cutoff: 11.0V) |
| DFRobot DC-DC buck converter | DigiKey 1738-DFR1202-ND; 12V → 5V for logic |
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

## Build Commands

All commands run on the Raspberry Pi from the repo root, after sourcing ROS 2:

```bash
source /opt/ros/jazzy/setup.bash

# Build both packages
colcon build --packages-select portamail_coordinator portamail_navigator

# Source workspace after build
source install/setup.bash
```

First-time Pi setup (installs ROS 2 Jazzy, dependencies, Docker for micro-ROS agent):
```bash
./setup_and_build.sh
```

## Running the System

### Simulation (desktop, no hardware)
```bash
# Terminal 1: Navigator with mock driver
ros2 launch portamail_navigator mapping.launch.py use_mock_driver:=true use_real_lidar:=false

# Terminal 2: Coordinator in mapping mode
ros2 launch portamail_coordinator bringup.launch.py mode:=mapping
```

### Physical Robot (Raspberry Pi)
```bash
# Terminal 1: Micro-ROS agent (bridges Teensy USB serial to ROS 2 topics)
docker run -it --rm --net=host --privileged -v /dev:/dev \
    microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

# Terminal 2: Hardware stack
# Without IMU (current state — BNO055 not yet wired):
ros2 launch portamail_navigator hardware.launch.py use_imu:=false

# With IMU (once physically wired):
ros2 launch portamail_navigator hardware.launch.py use_imu:=true

# Terminal 3: SLAM mapping or autonomous navigation
ros2 launch portamail_coordinator bringup.launch.py mode:=mapping
ros2 launch portamail_coordinator bringup.launch.py mode:=navigation
```

**Foxglove visualization**: `ws://<robot-ip>:8765` (started automatically by `mapping.launch.py`)

## Architecture

### Package: `portamail_navigator`

Low-level hardware abstraction and sensor stack.

- **`firmware/teensy_driver/teensy_driver.ino`** — Micro-ROS sketch for Teensy 4.0. Subscribes to `/cmd_vel`; publishes `/wheel/odom` (20 Hz), `/imu/data` (20 Hz, BNO055 via I2C pins 18/19), and `/ultrasonic/range` (10 Hz). Motor driver controlled via INA/INB+PWM. FIT0186 encoders on interrupt pins. 1-second safety timeout stops motors if no `cmd_vel` arrives. Requires Arduino libraries: `Adafruit BNO055`, `Adafruit Unified Sensor`.

- **`src/mock_driver.cpp`** — Simulated differential-drive node (`mock_teensy_driver`). Subscribes to `cmd_vel`, integrates kinematics at 20 Hz, publishes `odom` + the `odom→base_link` TF. Use on desktop in place of real hardware.

- **`config/ekf.yaml`** — `robot_localization` EKF fusing `/wheel/odom` + `/imu/data` (BNO055). Use when IMU is physically connected.

- **`config/ekf_no_imu.yaml`** — EKF with wheel odometry only. Default until BNO055 is wired.

- **`config/slam.yaml`** — SLAM Toolbox parameters (5 cm resolution, Ceres solver, loop closure enabled).

- **`launch/hardware.launch.py`** — Brings up robot_state_publisher (from URDF), Micro-ROS agent (Teensy bridge), RPLIDAR A2M12, and EKF. The `use_imu` argument (default `false`) selects which EKF config is loaded. There is **no Pi-side IMU node** — the BNO055 is wired to the Teensy via I2C; the Teensy firmware publishes `/imu/data` over micro-ROS when the IMU is connected.

- **`launch/mapping.launch.py`** — Includes `hardware.launch.py`, optionally starts `mock_driver`, Foxglove bridge.

- **`urdf/portamail.urdf`** — Robot description with full TF tree. Includes `laser`, `imu_link`, and `ultrasonic_link` fixed frames attached to `base_link`. **Sensor frame offsets are estimates and must be measured on the physical robot before SLAM is run.**

- **`src/robot_state_publisher.cpp`** — Not compiled (not in CMakeLists). Dead code; sensor TF frames are now in the URDF.

### Package: `portamail_coordinator`

High-level mission logic.

- **`src/navigation_coordinator.cpp`** — Single ROS 2 node (`navigation_coordinator`). Two modes (set by `start_mode` parameter):
  - **MAPPING**: Accepts only `save_map` command → calls `/slam_toolbox/save_map` service.
  - **NAVIGATION**: Accepts named location strings from `locations.yaml` → sends Nav2 `NavigateToPose` action goals.
  - Subscribes to `user_delivery_request` (`std_msgs/String`), publishes to `system_status` (`std_msgs/String`).

- **`config/locations.yaml`** — Named waypoints in `map` frame (x, y, orientation.w). Edit to add delivery destinations. Currently: `mailroom`, `office_101`, `office_102`, `lobby`.

- **`launch/bringup.launch.py`** — Launches coordinator with `mode` argument (`mapping` | `navigation`).

### Key ROS Topics / Actions
| Topic / Action | Type | Source → Sink |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / joystick → Teensy (micro-ROS USB) |
| `/wheel/odom` | `nav_msgs/Odometry` | Teensy encoders → EKF (micro-ROS USB) |
| `/imu/data` | `sensor_msgs/Imu` | Teensy BNO055 I2C → EKF (micro-ROS USB) |
| `/ultrasonic/range` | `sensor_msgs/Range` | Teensy HC-SR04 → Nav2 costmap (micro-ROS USB) |
| `/scan` | `sensor_msgs/LaserScan` | RPLIDAR A2M12 (`/dev/serial0`) → SLAM Toolbox |
| `/odom` | `nav_msgs/Odometry` | Mock driver → Nav2 (simulation only) |
| `user_delivery_request` | `std_msgs/String` | UI → coordinator |
| `system_status` | `std_msgs/String` | coordinator → UI |
| `/navigate_to_pose` | Nav2 action | coordinator → Nav2 |
| `/slam_toolbox/save_map` | service | coordinator → SLAM Toolbox |

### TF Frame Tree
`map` → `odom` → `base_link` → `laser`

The EKF (real hardware) or `mock_driver` (simulation) publishes the `odom→base_link` transform. SLAM Toolbox (mapping mode) or AMCL (navigation mode) publishes `map→odom`.

## Critical Hardware Constants (Firmware)

Constants in `firmware/teensy_driver/teensy_driver.ino` that must be calibrated on the physical robot:

| Constant | Current Value | Basis | Notes |
|---|---|---|---|
| `WHEEL_RADIUS` | `0.03556` m | 2.8" wheels ÷ 2 | Verify on actual tire |
| `WHEEL_BASE` | `0.20` m | Estimated | **Measure the actual track width** |
| `TICKS_PER_REV` | `720.0` | 8 PPR × 90:1 gear = 720 (single-edge) | **Calibrate physically**: drive exactly 1 m, count ticks |
| `MAX_SPEED_MPS` | `0.89` | 2 MPH limit | Matches SMART Goal #11 |
| `PWM_MIN` | `70` | Motor dead zone | Tune per motor |

**EKF sensor selection**: Pass `use_imu:=false` (default) for odometry-only fusion. Once BNO055 is physically wired to the Teensy's I2C pins and the Teensy firmware publishes `/imu/data`, pass `use_imu:=true` to load the full fusion config.

**Pi UART setup**: The RPLIDAR A2M12 is wired to Pi GPIO14 (TX) / GPIO15 (RX). Before first use, enable UART and disable serial console on the Pi:
```bash
# In /boot/firmware/config.txt, add:
enable_uart=1
# Then disable the serial console in raspi-config or by editing /boot/firmware/cmdline.txt
# (remove: console=serial0,115200)
```

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
- **Maps saved to**: `/home/ubuntu/PortaMailCapstone/maps/` (configurable via `map_save_path` parameter)
