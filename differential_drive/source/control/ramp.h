#pragma once

namespace diff_drive_control {

struct RampResult {
  float value;
  bool limited;
};

RampResult ramp_update(float current, float target, float max_rate_per_sec, float dt_s);

} // namespace diff_drive_control
