#pragma once

#include <stdint.h>

namespace diff_drive_safety {

enum SafetyState : uint8_t {
  BOOT_STOP = 0,
  RUN = 1,
  ESTOP_LATCHED = 2,
  FAULT = 3
};

constexpr uint32_t FAULT_FLAG_ESTOP = 1u << 0;
constexpr uint32_t FAULT_FLAG_FAULT = 1u << 1;
constexpr uint32_t FAULT_FLAG_TIMEOUT = 1u << 2;
constexpr uint32_t FAULT_FLAG_BOOT_STOP = 1u << 3;
constexpr uint32_t FAULT_FLAG_SATURATED = 1u << 4;
constexpr uint32_t FAULT_FLAG_RAMP_LIMITED = 1u << 5;

class SafetyManager {
public:
  SafetyManager();
  void reset();
  void update(uint16_t cmd_age_ms,
              bool cmd_vel_received,
              bool cmd_estop,
              bool cmd_clear_estop,
              bool fault_pin_active);

  SafetyState state() const;
  bool timeout_active() const;
  uint32_t fault_flags() const;
  uint16_t timeout_count() const;
  bool have_cmd() const;

private:
  SafetyState _state;
  bool _estop_latched;
  bool _fault_latched;
  bool _have_cmd;
  bool _timeout_active;
  uint16_t _timeout_count;
};

} // namespace diff_drive_safety
