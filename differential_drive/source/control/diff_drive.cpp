#include "diff_drive.h"

#include <math.h>

#include "../config/dimensions.h"
#include "ramp.h"

namespace diff_drive_control {

namespace {
float clamp_float(float value, float min_v, float max_v) {
  if (value < min_v) {
    return min_v;
  }
  if (value > max_v) {
    return max_v;
  }
  return value;
}

float clamp_abs(float value, float max_abs) {
  if (value > max_abs) {
    return max_abs;
  }
  if (value < -max_abs) {
    return -max_abs;
  }
  return value;
}
} // namespace

DiffDriveController::DiffDriveController() : _vL_mps(0.0f), _vR_mps(0.0f) {}

void DiffDriveController::reset() {
  _vL_mps = 0.0f;
  _vR_mps = 0.0f;
}

WheelTargets DiffDriveController::update(float v_cmd_mps, float w_cmd_radps, float dt_s) {
  using namespace diff_drive_config;

  const float v_clamped = clamp_float(v_cmd_mps, -MAX_V_MPS, MAX_V_MPS);
  const float w_clamped = clamp_float(w_cmd_radps, -MAX_W_RADPS, MAX_W_RADPS);

  const float half_track = TRACK_WIDTH_M * 0.5f;
  float vL_target = v_clamped - (w_clamped * half_track);
  float vR_target = v_clamped + (w_clamped * half_track);

  bool saturated = false;
  if (fabsf(vL_target) > MAX_WHEEL_MPS) {
    vL_target = clamp_abs(vL_target, MAX_WHEEL_MPS);
    saturated = true;
  }
  if (fabsf(vR_target) > MAX_WHEEL_MPS) {
    vR_target = clamp_abs(vR_target, MAX_WHEEL_MPS);
    saturated = true;
  }

  const RampResult rampL = ramp_update(_vL_mps, vL_target, MAX_WHEEL_ACCEL_MPS2, dt_s);
  const RampResult rampR = ramp_update(_vR_mps, vR_target, MAX_WHEEL_ACCEL_MPS2, dt_s);

  _vL_mps = rampL.value;
  _vR_mps = rampR.value;

  const bool ramp_limited = rampL.limited || rampR.limited;

  WheelTargets out{};
  out.vL_mps = _vL_mps;
  out.vR_mps = _vR_mps;
  out.applied_v_mps = v_clamped;
  out.applied_w_radps = w_clamped;
  out.saturated = saturated;
  out.ramp_limited = ramp_limited;
  return out;
}

} // namespace diff_drive_control
