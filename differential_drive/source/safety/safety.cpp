#include "safety.h"

#include "../config/dimensions.h"

namespace diff_drive_safety {

SafetyManager::SafetyManager()
    : _state(BOOT_STOP),
      _estop_latched(false),
      _fault_latched(false),
      _have_cmd(false),
      _timeout_active(false),
      _timeout_count(0) {}

void SafetyManager::reset() {
  _state = BOOT_STOP;
  _estop_latched = false;
  _fault_latched = false;
  _have_cmd = false;
  _timeout_active = false;
  _timeout_count = 0;
}

void SafetyManager::update(uint16_t cmd_age_ms,
                           bool cmd_vel_received,
                           bool cmd_estop,
                           bool cmd_clear_estop,
                           bool fault_pin_active) {
  using namespace diff_drive_config;

  if (cmd_vel_received) {
    _have_cmd = true;
  }

  if (cmd_estop) {
    _estop_latched = true;
  }

  if (fault_pin_active) {
    _fault_latched = true;
  }

  if (cmd_clear_estop && !fault_pin_active) {
    _fault_latched = false;
  }

  if (cmd_clear_estop && !_fault_latched) {
    _estop_latched = false;
  }

  const bool new_timeout = _have_cmd && (cmd_age_ms > CMD_TIMEOUT_MS);
  if (new_timeout && !_timeout_active) {
    _timeout_count++;
  }
  _timeout_active = new_timeout;

  if (_fault_latched) {
    _state = FAULT;
  } else if (_estop_latched) {
    _state = ESTOP_LATCHED;
  } else if (!_have_cmd) {
    _state = BOOT_STOP;
  } else {
    _state = RUN;
  }
}

SafetyState SafetyManager::state() const {
  return _state;
}

bool SafetyManager::timeout_active() const {
  return _timeout_active;
}

uint32_t SafetyManager::fault_flags() const {
  uint32_t flags = 0u;
  if (_estop_latched) {
    flags |= FAULT_FLAG_ESTOP;
  }
  if (_fault_latched) {
    flags |= FAULT_FLAG_FAULT;
  }
  if (_timeout_active) {
    flags |= FAULT_FLAG_TIMEOUT;
  }
  if (_state == BOOT_STOP) {
    flags |= FAULT_FLAG_BOOT_STOP;
  }
  return flags;
}

uint16_t SafetyManager::timeout_count() const {
  return _timeout_count;
}

bool SafetyManager::have_cmd() const {
  return _have_cmd;
}

} // namespace diff_drive_safety
