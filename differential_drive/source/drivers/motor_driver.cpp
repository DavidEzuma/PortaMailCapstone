#include "motor_driver.h"

#include "../config/build_mode.h"

#if DIFF_DRIVE_DEBUG_SW
#include <iostream>
#else
#include <Arduino.h>
#endif
#include <math.h>

#include "../config/dimensions.h"

namespace diff_drive_drivers {

MotorDriver::MotorDriver() : _valid(false) {}

bool MotorDriver::valid() const {
  return _valid;
}

void MotorDriver::begin() {
  using namespace diff_drive_config;

  _valid = (PWM_L >= 0) && (DIR_L_A >= 0) && (DIR_L_B >= 0) &&
           (PWM_R >= 0) && (DIR_R_A >= 0) && (DIR_R_B >= 0) && (PWM_MAX > 0);
#if DIFF_DRIVE_DEBUG_SW
  _valid = true;
  return;
#else
  if (!_valid) {
    if (ENABLE >= 0) {
      pinMode(ENABLE, OUTPUT);
      digitalWrite(ENABLE, LOW);
    }
    return;
  }

#if !DIFF_DRIVE_DEBUG_SW
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L_A, OUTPUT);
  pinMode(DIR_L_B, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R_A, OUTPUT);
  pinMode(DIR_R_B, OUTPUT);
  if (ENABLE >= 0) {
    pinMode(ENABLE, OUTPUT);
    digitalWrite(ENABLE, LOW);
  }

#if defined(TEENSYDUINO)
  analogWriteFrequency(PWM_L, PWM_FREQ_HZ);
  analogWriteFrequency(PWM_R, PWM_FREQ_HZ);
#endif

  stop();
#endif
#endif
}

void MotorDriver::stop() {
  using namespace diff_drive_config;
  if (!_valid) {
    return;
  }

#if !DIFF_DRIVE_DEBUG_SW
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  digitalWrite(DIR_L_A, LOW);
  digitalWrite(DIR_L_B, LOW);
  digitalWrite(DIR_R_A, LOW);
  digitalWrite(DIR_R_B, LOW);
  if (ENABLE >= 0) {
    digitalWrite(ENABLE, LOW);
  }
#else
  std::cout << "[MOTOR] stop PWM_L=0 PWM_R=0 DIR_L=00 DIR_R=00\n";
#endif
}

void MotorDriver::set_wheels_mps(float vL_mps, float vR_mps, bool enable) {
  using namespace diff_drive_config;

  if (!_valid) {
    return;
  }

#if !DIFF_DRIVE_DEBUG_SW
  if (ENABLE >= 0) {
    digitalWrite(ENABLE, enable ? HIGH : LOW);
  }
#endif

  if (!enable) {
#if !DIFF_DRIVE_DEBUG_SW
    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, 0);
#else
    std::cout << "[MOTOR] disabled PWM_L=0 PWM_R=0 DIR_L=00 DIR_R=00\n";
#endif
    return;
  }

  set_single(vL_mps, PWM_L, DIR_L_A, DIR_L_B, INVERT_LEFT);
  set_single(vR_mps, PWM_R, DIR_R_A, DIR_R_B, INVERT_RIGHT);
}

void MotorDriver::set_single(float v_mps, int pwm_pin, int dir_a_pin, int dir_b_pin, bool invert) {
  using namespace diff_drive_config;

  float speed = v_mps;
  if (invert) {
    speed = -speed;
  }

  const bool forward = (speed >= 0.0f);
  const float abs_speed = fabsf(speed);

  float norm = abs_speed / MAX_WHEEL_MPS;
  if (norm > 1.0f) {
    norm = 1.0f;
  }

  uint16_t pwm = 0;
  if (norm <= DEADBAND_NORM) {
    pwm = 0;
  } else {
    const float scaled = (norm - DEADBAND_NORM) / (1.0f - DEADBAND_NORM);
    const float pwm_f = MIN_START_PWM + scaled * (static_cast<float>(PWM_MAX - MIN_START_PWM));
    if (pwm_f >= static_cast<float>(PWM_MAX)) {
      pwm = PWM_MAX;
    } else if (pwm_f <= static_cast<float>(MIN_START_PWM)) {
      pwm = MIN_START_PWM;
    } else {
      pwm = static_cast<uint16_t>(pwm_f + 0.5f);
    }
  }

#if !DIFF_DRIVE_DEBUG_SW
  if (pwm == 0) {
    // EN low = coast on L298N; both inputs low is a safe idle state.
    digitalWrite(dir_a_pin, LOW);
    digitalWrite(dir_b_pin, LOW);
  } else {
    digitalWrite(dir_a_pin, forward ? HIGH : LOW);
    digitalWrite(dir_b_pin, forward ? LOW : HIGH);
  }
  analogWrite(pwm_pin, pwm);
#else
  if (pwm == 0) {
    std::cout << "[MOTOR] PWM=" << pwm << " DIR_A=0 DIR_B=0\n";
  } else {
    std::cout << "[MOTOR] PWM=" << pwm << " DIR_A=" << (forward ? 1 : 0)
              << " DIR_B=" << (forward ? 0 : 1) << "\n";
  }
#endif
}

} // namespace diff_drive_drivers
