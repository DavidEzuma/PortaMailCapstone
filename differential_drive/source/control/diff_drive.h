#pragma once

#include <stdint.h>

namespace diff_drive_control {

struct WheelTargets {
  float vL_mps;
  float vR_mps;
  float applied_v_mps;
  float applied_w_radps;
  bool saturated;
  bool ramp_limited;
};

class DiffDriveController {
public:
  DiffDriveController();
  void reset();
  WheelTargets update(float v_cmd_mps, float w_cmd_radps, float dt_s);

private:
  float _vL_mps;
  float _vR_mps;
};

} // namespace diff_drive_control
