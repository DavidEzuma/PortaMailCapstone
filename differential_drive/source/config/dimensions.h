#pragma once

#include <stdint.h>

namespace diff_drive_config {

constexpr float TRACK_WIDTH_M = 0.35f;
constexpr float WHEEL_RADIUS_M = 0.05f;

constexpr float MAX_V_MPS = 1.0f;
constexpr float MAX_W_RADPS = 2.0f;
constexpr float MAX_WHEEL_MPS = 1.2f;
constexpr float MAX_WHEEL_ACCEL_MPS2 = 2.5f;

constexpr uint32_t CMD_TIMEOUT_MS = 200U;
constexpr uint16_t STATUS_RATE_HZ = 10U;

constexpr uint32_t SERIAL_BAUD = 921600U; // Use 115200 if USB-serial is unstable.

constexpr uint32_t PWM_FREQ_HZ = 20000U;
constexpr uint16_t PWM_MAX = 255U;
constexpr float DEADBAND_NORM = 0.05f;
constexpr uint16_t MIN_START_PWM = 30U;

constexpr bool INVERT_LEFT = false;
constexpr bool INVERT_RIGHT = false;

// Hardware pin map (active only in HARDWARE mode).
// L298N wiring:
//   Left motor  -> ENA(PWM_L), IN1(DIR_L_A), IN2(DIR_L_B)
//   Right motor -> ENB(PWM_R), IN3(DIR_R_A), IN4(DIR_R_B)
constexpr int PWM_L = 2;
constexpr int DIR_L_A = 4;
constexpr int DIR_L_B = 5;
constexpr int PWM_R = 3;
constexpr int DIR_R_A = 6;
constexpr int DIR_R_B = 7;
constexpr int ENABLE = -1; // Unused; PWM drives ENA/ENB directly.

constexpr int DRIVER_FAULT_PIN = -1; // -1 if unused

} // namespace diff_drive_config
