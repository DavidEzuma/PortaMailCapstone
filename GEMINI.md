# PortaMail Project Context & Mandates

This file serves as the primary reference for the PortaMail project. Adhere to these mandates and configurations for all development tasks.

## Project Overview
PortaMail is an autonomous indoor mail-delivery robot built for hospital/corporate environments. It uses SLAM for mapping and Nav2 for autonomous navigation.

- **Lead Developers:** David Ezuma (Software/ROS 2), Henry Lovelace (Firmware/Hardware)
- **Target OS:** Ubuntu 24.04 (Noble)
- **ROS 2 Distribution:** Jazzy Jalisco

## Core Tech Stack
- **Compute:** Raspberry Pi 5 (8GB) + Teensy 4.0 (Low-level MCU)
- **Communication:** micro-ROS over USB Serial (`/dev/ttyACM0` @ 115200 baud)
- **Sensors:** 
  - SLAMTEC RPLIDAR A2M12 (`/dev/ttyUSB0` @ 256000 baud, scan_mode: Sensitivity)
  - Adafruit BNO055 IMU (I2C to Teensy)
  - HC-SR04 Ultrasonic Sensor (to Teensy)
- **Drivetrain:** Differential drive (DFRobot FIT0186 gearmotors + VNH5019A-E drivers)
- **UI:** Flask-based web interface displayed on a 7" Waveshare touchscreen (Chromium kiosk mode)

## Workspace Structure
- `src/portamail_navigator`: Hardware abstraction, sensors (LiDAR, IMU, Encoders), and SLAM.
- `src/portamail_coordinator`: High-level mission logic, waypoint management, and UI bridge.
- `src/sllidar_ros2`: Driver for the RPLIDAR.
- `LCD_web/portamail_ui`: Flask application for the touchscreen GUI.

## Critical Mandates & Conventions

### 1. Hardware Safety
- **Safety Timeout:** Teensy firmware stops motors if `/cmd_vel` is not received for > 1 second.
- **Voltage Warning:** Teensy 4.0 GPIO is 3.3V only. Ensure 5V sensors (like HC-SR04 Echo) use level shifters or voltage dividers.

### 2. Configuration & Calibration
- **LiDAR:** Must use `scan_mode:=Sensitivity` for the A2M12 model.
- **USB Mapping:** LiDAR is typically `/dev/ttyUSB0`, Teensy is `/dev/ttyACM0`.
- **Constants (Teensy):**
  - `WHEEL_RADIUS`: 0.03556 m
  - `WHEEL_BASE`: 0.20 m (Estimate - Verify physically)
  - `TICKS_PER_REV`: 720.0 (Verify via 1m travel test)
  - `MAX_SPEED_MPS`: 0.89 m/s (2 MPH)

### 3. Build & Run Workflow
- **Build:** `colcon build --packages-select portamail_coordinator portamail_navigator`
- **Full System Start:** `./launch_portamail.sh`
- **Mapping Only:** `ros2 launch portamail_navigator mapping.launch.py`

### 4. URDF & TFs
- **Base Frame:** `base_link` (centered on drive axle)
- **TF Tree:** `map` -> `odom` -> `base_link` -> `laser`/`imu_link`/`ultrasonic_link`
- **Sensor Offsets:** Current offsets in `src/portamail_navigator/urdf/portamail.urdf` are estimates and must be measured physically.

## Active Tasks & Checklist
- [ ] Calibrate `WHEEL_BASE` and `TICKS_PER_REV`.
- [ ] Verify Ultrasonic ECHO pin wiring and voltage levels.
- [ ] Wire and enable BNO055 IMU on Teensy I2C.
- [ ] Update URDF sensor offsets with physical measurements.
