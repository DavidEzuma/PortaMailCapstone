# Teensy Differential Drive Executor (Phase 1)

## Purpose and Scope
This module is a low-level, open-loop motor executor for a differential-drive base. It accepts raw velocity commands (`v`, `w`) over USB serial and drives left/right motor outputs (PWM + DIR) with safety gating, rate limits, and status telemetry. It does **not** perform SLAM, localization, odometry, encoder processing, IMU processing, mapping, or path planning.

## Protocol Overview
Binary framed packets over USB serial:

```
[0xAA][0x55]
[version:uint8 = 0x01]
[msg_type:uint8]
[payload_len:uint8]
[seq:uint16 little-endian]
[payload:N bytes]
[crc16:uint16 little-endian]
```

CRC16-CCITT-FALSE is computed over:
```
[version, msg_type, payload_len, seq(2 bytes), payload(N bytes)]
```
SOF bytes are **not** included in the CRC.

### CRC16-CCITT-FALSE Parameters
- poly = 0x1021
- init = 0xFFFF
- refin = false
- refout = false
- xorout = 0x0000
- Test vector: "123456789" → 0x29B1

## Message Types and Payloads
All multi-byte fields are **little-endian**.

- `CMD_VEL (0x01)` Pi → Teensy
  - payload_len = 8
  - payload = `float32 v_mps` + `float32 w_radps`

- `CMD_ESTOP (0x02)` Pi → Teensy
  - payload_len = 0

- `CMD_CLEAR_ESTOP (0x03)` Pi → Teensy
  - payload_len = 0

- `STATUS (0x10)` Teensy → Pi only
  - payload_len = 34

STATUS payload (exact 34 bytes):
```
uint16 seq_ack
float  applied_v_mps
float  applied_w_radps
float  target_vL_mps
float  target_vR_mps
uint32 fault_flags
uint16 cmd_age_ms
uint16 crc_fail_count
uint16 resync_count
uint16 unknown_count
uint16 seq_drop_count
uint16 timeout_count
```

## Safety Behavior
- Initial state is `BOOT_STOP`.
- `BOOT_STOP → RUN` only after the **first valid CMD_VEL**, and only if not ESTOP or FAULT.
- Timeout (`cmd_age_ms > 200`) is **not latched** and does **not** force ESTOP; it simply commands zero output until a new valid CMD_VEL arrives.
- `CMD_ESTOP` latches `ESTOP_LATCHED` until `CMD_CLEAR_ESTOP`.
- `FAULT` is latched when the driver fault pin is active; it clears only when the fault pin deasserts **and** `CMD_CLEAR_ESTOP` is received.

## Tunables (config/dimensions.h)
- Geometry: `TRACK_WIDTH_M`, `WHEEL_RADIUS_M`
- Command limits: `MAX_V_MPS`, `MAX_W_RADPS`
- Wheel limits: `MAX_WHEEL_MPS`, `MAX_WHEEL_ACCEL_MPS2`
- Timing: `CMD_TIMEOUT_MS`, `STATUS_RATE_HZ`
- Serial: `SERIAL_BAUD` (default 921600; use 115200 if needed)
- PWM: `PWM_FREQ_HZ`, `PWM_MAX`, `DEADBAND_NORM`, `MIN_START_PWM`
- Polarity: `INVERT_LEFT`, `INVERT_RIGHT`
- Pins: `PWM_L`, `DIR_L`, `PWM_R`, `DIR_R`, `ENABLE`, `DRIVER_FAULT_PIN`

If any required motor pins are set to `-1`, the driver stays disabled (fail-safe).

## Bring-up Checklist
1. Set correct pin assignments in `config/dimensions.h`.
2. Confirm `PWM_FREQ_HZ` matches your motor driver.
3. Verify `INVERT_LEFT/INVERT_RIGHT` for correct wheel direction.
4. If using a fault pin, set `DRIVER_FAULT_PIN` and confirm its active level.
5. Connect over USB serial at `SERIAL_BAUD` (fallback 115200).
6. Send `CMD_VEL` at ~50 Hz.
7. Verify `STATUS` at 10 Hz with correct `seq_ack` and `cmd_age_ms`.

## Minimal Python CMD_VEL Sender (no ROS)
```python
import struct
import serial

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.01)
seq = 0
v = 0.5  # m/s
w = 0.2  # rad/s

payload = struct.pack('<ff', v, w)
header = struct.pack('<BBB', 0x01, 0x01, len(payload)) + struct.pack('<H', seq)
crc = crc16_ccitt_false(header + payload)
frame = b'\xAA\x55' + header + payload + struct.pack('<H', crc)
ser.write(frame)
```

## Architecture Notes
- **Protocol**: binary framing + CRC validation + parser counters.
- **Safety**: state machine, timeout behavior, and latched ESTOP/FAULT handling.
- **Control**: command clamping and slew-limited wheel targets.
- **Driver**: PWM + DIR motor output with deadband and minimum-start PWM.

This interface cleanly supports later encoder/IMU integration: higher-level systems can add feedback and state estimation on the Raspberry Pi without changing the `CMD_VEL`/`STATUS` protocol or the low-level executor wiring.
