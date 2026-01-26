#include "ramp.h"

#include <math.h>

namespace diff_drive_control {

RampResult ramp_update(float current, float target, float max_rate_per_sec, float dt_s) {
  if (dt_s <= 0.0f || max_rate_per_sec <= 0.0f) {
    return {target, false};
  }

  const float delta = target - current;
  const float max_step = max_rate_per_sec * dt_s;
  if (fabsf(delta) <= max_step) {
    return {target, false};
  }

  const float step = (delta > 0.0f) ? max_step : -max_step;
  return {current + step, true};
}

} // namespace diff_drive_control
