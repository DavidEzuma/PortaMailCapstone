#pragma once

namespace diff_drive_drivers {

class MotorDriver {
public:
  MotorDriver();
  void begin();
  void set_wheels_mps(float vL_mps, float vR_mps, bool enable);
  void stop();
  bool valid() const;

private:
  bool _valid;
  void set_single(float v_mps, int pwm_pin, int dir_a_pin, int dir_b_pin, bool invert);
};

} // namespace diff_drive_drivers
