# Research: Motor Response, ESP32 Comms, SLAM Quality, Launch Bloat (2026-03-30)

## Problem Statement

Comprehensive codebase audit across four areas to find and implement all
improvements possible given the current hardware state (ESP32 + L298N motors
+ RPLIDAR A2M12 connected; no wheels/body/IMU yet).

## Requirements

- Implement everything that doesn't assume uncalibrated physical hardware
- Leave constants with `*** CALIBRATE ***` markers for physical-only tuning
- Do not add any node starts that aren't needed for current testing
- Include ESP32 hardware reset on every mode enter/leave

---

## Findings

### Relevant Files

| File | Purpose | Key Changes |
|---|---|---|
| `firmware/esp32_driver/esp32_driver.ino` | Motor/sensor firmware | 7 improvements |
| `launch/hardware.launch.py` | Hardware stack launch | Baud constant clarified |
| `config/slam.yaml` | SLAM Toolbox tuning | Rate sync + loop closure tightened |
| `config/ekf_no_imu.yaml` | EKF wheel-only config | Covariance is set in firmware (not this file) |
| `launch/mapping.launch.py` | Mapping stack launch | Foxglove optional; SLAM delay configurable; autosave 120s |
| `launch_portamail.sh` | Top-level startup | ESP32 RTS reset; faster polling |

---

## Implemented Changes

### Area 1: Motor Response

**1. Per-motor PWM dead zone (`PWM_MIN_LEFT`, `PWM_MIN_RIGHT`)**
- Both set to 70 with clear `*** CALIBRATE per motor ***` comment
- `setMotor()` accepts `pwm_min` parameter so each motor uses its own threshold
- Physical robot must be used to find minimum PWM per wheel (apply power, find threshold, add 5)

**2. Velocity normalization before clamping**
- Previous: `target_left = clamp(lin - ang×wb/2)` — asymmetric clamp reduced turn rate at speed
- Fix: compute raw left/right, find `max_raw = max(|left|, |right|)`, scale both by `MAX_SPEED/max_raw` if over limit
- Preserves turn rate at maximum linear speed

**3. Encoder math — explicit float cast**
- Previous: `(lticks / TICKS_PER_REV) * circumference` — integer division lost sub-mm precision on small tick counts
- Fix: `((float)lticks / TICKS_PER_REV)` — forces float division immediately

**4. PWM frequency increased from 5 kHz → 10 kHz**
- Better resolution at low duty cycles; smoother torque at low speeds; still within L298N spec

**5. Odom covariance values relaxed**
- Previous: `pose[0]=0.01, pose[35]=0.10, twist[0]=0.02, twist[35]=0.20` (too tight — trusts encoders more than reality)
- Fix: `pose[0]=0.05, pose[35]=0.25, twist[0]=0.05, twist[35]=0.50`
- More conservative: EKF weighs odometry less vs SLAM; cleaner loop closures

### Area 2: ESP32 ↔ Pi Communication

**6. Ping timeout reduced from `500ms × 3` → `1000ms × 1`**
- Previous 1500ms window with 3 retries could false-trigger during SLAM loop closure (Pi CPU spike)
- New 1000ms single-shot: same total window, no retry overhead, less likely to false-disconnect

**7. ESP32 hardware reset via RTS on mode exit**
- `_reset_esp32()` shell function in `launch_portamail.sh`
- Finds serial port by stable by-id path → falls back to ttyUSB1, then ttyUSB0
- Pulses RTS LOW for 150ms via Python `serial.Serial.setRTS()` — triggers EN reset through CP2102 auto-reset circuit
- Called at end of `_kill_ros()` — fires after micro_ros_agent has released the port
- 1s post-reset delay before function returns; next mode start connects to a freshly booted MCU

**8. SERIAL_BAUD constant renamed to `LIDAR_BAUD`**
- Was named `SERIAL_BAUD = 256000` but used only for the LiDAR node — misleading since micro-ROS agent uses 115200
- Renamed + commented to prevent confusion when debugging serial issues

### Area 3: SLAM / Map Quality

**9. `minimum_time_interval` synced to odom rate**
- Previous: `0.1` (100ms) vs odom at 50ms and TF at 50ms — SLAM update rate was 2× slower than available data
- Fix: `0.05` — SLAM now updates at 20 Hz, matching odom and TF publish rates; tighter scan registration

**10. Loop closure variance tightened**
- Previous: `loop_match_maximum_variance_coarse: 3.0` — accepted coarse matches with ~1.7m standard deviation (bad for corridors with repetitive doorways)
- Fix: `1.5` + raise `loop_match_minimum_response_coarse: 0.50` (was 0.35)
- Prevents false closures when passing similar-looking corridor sections multiple times

**11. Map autosave interval increased 30s → 120s**
- SLAM Toolbox locks the map buffer during each save (1-2s) causing a brief odom/SLAM gap
- 30s interval = SLAM stalled every 30s during mapping; 120s is acceptable for session-length maps

### Area 4: Launch Bloat & Process Sequencing

**12. Foxglove bridge disabled by default in mapping mode**
- Was always started, consuming ~5-10% CPU on the Pi via WebSocket + topic introspection
- Now off by default; enable with `enable_foxglove:=true` for remote visualization sessions

**13. SLAM Toolbox configure delay made configurable and reduced**
- Previous: hardcoded `period=2.0` — always waited 2s regardless of actual TF readiness
- Fix: `slam_configure_delay_sec` arg, default `0.5s` — robot_state_publisher starts fast; 0.5s is enough
- Override with `slam_configure_delay_sec:=2.0` if TF errors appear on startup

**14. Mode-select poll rate doubled (1s → 0.5s)**
- Mode selection feels more responsive; still well within Flask server capacity

**15. Mode-monitor poll rate doubled (2s → 1s)**
- Back-button detection latency halved; mode exit is more responsive

---

## Pending (Physical Calibration Required)

These cannot be fixed without the assembled robot:

| Item | File | What To Do |
|---|---|---|
| `PWM_MIN_LEFT` / `PWM_MIN_RIGHT` | `esp32_driver.ino` | Find min PWM per motor on bench, add 5, update |
| `WHEEL_BASE` | `esp32_driver.ino` | Measure center-to-center track width in meters |
| `TICKS_PER_REV` | `esp32_driver.ino` | Drive 1m, count ticks, compute per CLAUDE.md §4 |
| URDF laser frame offset | `urdf/portamail.urdf` | Measure forward/height offset of LiDAR from axle |

---

## Open Questions

1. Does the CP2102 on the specific ESP32 DevKit used support RTS-triggered reset?
   (Most DevKits do; verify by checking that the Arduino IDE auto-resets on upload)
2. Should the `_reset_esp32()` also be called on initial startup (before mapping mode),
   or only on mode transitions?

## Recommendations

- Reflash firmware after reviewing changes (all firmware edits are backward-compatible)
- After reflash, test motor asymmetry on bench (no wheels): apply small PWM, check both motors start at same threshold
- Set `enable_foxglove:=true` during visualization sessions, leave off for pure mapping runs
