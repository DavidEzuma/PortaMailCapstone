#include "config/build_mode.h"

#if DIFF_DRIVE_DEBUG_SW
#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>
#else
#include <Arduino.h>
#include <string.h>
#endif

#include "config/dimensions.h"
#include "control/diff_drive.h"
#include "drivers/motor_driver.h"
#include "safety/safety.h"
#include "status/status.h"

#if !DIFF_DRIVE_DEBUG_SW
#include "protocol/protocol.h"
#else
#include "protocol/protocol.h"
#endif

using diff_drive_config::DRIVER_FAULT_PIN;

#if !DIFF_DRIVE_DEBUG_SW
using diff_drive_config::SERIAL_BAUD;
using diff_drive_config::STATUS_RATE_HZ;
#endif

namespace {

float read_f32_le(const uint8_t *data) {
  float value = 0.0f;
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  uint8_t temp[4];
  temp[0] = data[3];
  temp[1] = data[2];
  temp[2] = data[1];
  temp[3] = data[0];
  memcpy(&value, temp, sizeof(float));
#else
  memcpy(&value, data, sizeof(float));
#endif
  return value;
}

uint16_t clamp_age_ms(uint32_t delta_ms) {
  if (delta_ms > 65535u) {
    return 65535u;
  }
  return static_cast<uint16_t>(delta_ms);
}

} // namespace

#if DIFF_DRIVE_DEBUG_SW
uint32_t now_ms() {
  using namespace std::chrono;
  static const steady_clock::time_point start = steady_clock::now();
  return static_cast<uint32_t>(
      duration_cast<milliseconds>(steady_clock::now() - start).count());
}

uint32_t now_us() {
  using namespace std::chrono;
  static const steady_clock::time_point start = steady_clock::now();
  return static_cast<uint32_t>(
      duration_cast<microseconds>(steady_clock::now() - start).count());
}
#endif

static diff_drive_drivers::MotorDriver motor;
static diff_drive_control::DiffDriveController controller;
static diff_drive_safety::SafetyManager safety;
static diff_drive_protocol::Frame rx_frame;

static float cmd_v_mps = 0.0f;
static float cmd_w_radps = 0.0f;
static bool cmd_vel_received = false;
static bool cmd_estop = false;
static bool cmd_clear_estop = false;
static uint32_t last_valid_cmd_ms = 0;
static uint16_t last_cmd_seq = 0;

static diff_drive_control::WheelTargets last_targets{};
static uint32_t last_motor_us = 0;
static uint32_t last_status_ms = 0;
static uint16_t status_seq = 0;

#if DIFF_DRIVE_DEBUG_SW
namespace {
class FileStream : public Stream {
public:
  explicit FileStream(const std::vector<uint8_t> &data) : _data(data), _pos(0) {}

  int available() override {
    if (_pos >= _data.size()) {
      return 0;
    }
    return static_cast<int>(_data.size() - _pos);
  }

  int read() override {
    if (_pos >= _data.size()) {
      return -1;
    }
    return static_cast<int>(_data[_pos++]);
  }

  size_t write(uint8_t) override {
    return 1;
  }

  size_t write(const uint8_t *, size_t size) override {
    return size;
  }

private:
  const std::vector<uint8_t> &_data;
  size_t _pos;
};

std::vector<uint8_t> read_file_bytes(const std::filesystem::path &path) {
  std::vector<uint8_t> data;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return data;
  }
  file.seekg(0, std::ios::end);
  const std::streamsize size = file.tellg();
  if (size <= 0) {
    return data;
  }
  file.seekg(0, std::ios::beg);
  data.resize(static_cast<size_t>(size));
  file.read(reinterpret_cast<char *>(data.data()), size);
  return data;
}

uint32_t parse_sleep_ms(const std::filesystem::path &path) {
  // Expected format: sleep_<ms>.txt (e.g., sleep_250ms.txt)
  const std::string name = path.stem().string();
  const std::string prefix = "sleep_";
  if (name.rfind(prefix, 0) != 0) {
    return 0;
  }
  const std::string value = name.substr(prefix.size());
  std::string digits;
  for (char c : value) {
    if (c >= '0' && c <= '9') {
      digits.push_back(c);
    }
  }
  if (digits.empty()) {
    return 0;
  }
  return static_cast<uint32_t>(std::stoul(digits));
}

std::string flags_to_string(uint32_t flags) {
  std::string out;
  if (flags & diff_drive_safety::FAULT_FLAG_ESTOP) {
    out += "ESTOP|";
  }
  if (flags & diff_drive_safety::FAULT_FLAG_FAULT) {
    out += "FAULT|";
  }
  if (flags & diff_drive_safety::FAULT_FLAG_TIMEOUT) {
    out += "TIMEOUT|";
  }
  if (flags & diff_drive_safety::FAULT_FLAG_BOOT_STOP) {
    out += "BOOT_STOP|";
  }
  if (flags & diff_drive_safety::FAULT_FLAG_SATURATED) {
    out += "SATURATED|";
  }
  if (flags & diff_drive_safety::FAULT_FLAG_RAMP_LIMITED) {
    out += "RAMP_LIMITED|";
  }
  if (out.empty()) {
    return "OK";
  }
  out.pop_back();
  return out;
}

void print_status_readable(const diff_drive_status::StatusInputs &status) {
  std::cout << "\nSTATUS:"
            << " seq_ack=" << status.seq_ack
            << " v=" << status.applied_v_mps
            << " w=" << status.applied_w_radps
            << " vL=" << status.target_vL_mps
            << " vR=" << status.target_vR_mps
            << " cmd_age_ms=" << status.cmd_age_ms
            << " flags=" << flags_to_string(status.fault_flags)
            << " crc_fail=" << status.crc_fail_count
            << " resync=" << status.resync_count
            << " unknown=" << status.unknown_count
            << " seq_drop=" << status.seq_drop_count
            << " timeout_cnt=" << status.timeout_count
            << "\n";
}
} // namespace

int main() {
  motor.begin();
  controller.reset();
  safety.reset();

  last_valid_cmd_ms = now_ms();
  last_motor_us = now_us();

  std::cout << "DIFF_DRIVE_DEBUG_SW: monitoring differential_drive/TEST for packets.\n";
  std::cout << "Supported: .bin frames, sleep_<ms>.txt delays\n";

  const std::filesystem::path test_dir = std::filesystem::path("TEST");
  std::unordered_set<std::string> processed;

  while (true) {
    std::vector<std::filesystem::path> entries;
    for (const auto &entry : std::filesystem::directory_iterator(test_dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      entries.push_back(entry.path());
    }

    std::sort(entries.begin(), entries.end());

    for (const auto &path : entries) {
      const std::string name = path.string();
      if (processed.find(name) != processed.end()) {
        continue;
      }

      if (path.extension() == ".txt") {
        const uint32_t sleep_ms = parse_sleep_ms(path);
        if (sleep_ms > 0) {
          std::cout << "[SLEEP] " << sleep_ms << " ms\n";
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        processed.insert(name);
        continue;
      }

      if (path.extension() != ".bin") {
        continue;
      }

      const std::vector<uint8_t> data = read_file_bytes(path);
      if (data.empty()) {
        continue;
      }

      FileStream stream(data);
      while (diff_drive_protocol::protocol_read_frame(stream, rx_frame)) {
        const uint32_t now_ms_val = now_ms();
        const uint32_t age_ms = now_ms_val - last_valid_cmd_ms;
        const uint16_t cmd_age_ms = clamp_age_ms(age_ms);

        std::cout << "\n[RX] seq=" << rx_frame.seq
                  << " cmd_age_ms=" << cmd_age_ms;

        if (rx_frame.msg_type == diff_drive_protocol::CMD_VEL) {
          const float v = read_f32_le(&rx_frame.payload[0]);
          const float w = read_f32_le(&rx_frame.payload[4]);
          std::cout << " CMD_VEL v=" << v << " w=" << w << "\n";

          cmd_v_mps = v;
          cmd_w_radps = w;
          last_valid_cmd_ms = now_ms_val;
          last_cmd_seq = rx_frame.seq;
          cmd_vel_received = true;
        } else if (rx_frame.msg_type == diff_drive_protocol::CMD_ESTOP) {
          std::cout << " CMD_ESTOP\n";
          cmd_estop = true;
        } else if (rx_frame.msg_type == diff_drive_protocol::CMD_CLEAR_ESTOP) {
          std::cout << " CMD_CLEAR_ESTOP\n";
          cmd_clear_estop = true;
        } else {
          std::cout << " (ignored)\n";
          continue;
        }

        const uint32_t age_ms_after = now_ms_val - last_valid_cmd_ms;
        const uint16_t cmd_age_ms_after = clamp_age_ms(age_ms_after);

        safety.update(cmd_age_ms_after, cmd_vel_received, cmd_estop, cmd_clear_estop, false);

        cmd_vel_received = false;
        cmd_estop = false;
        cmd_clear_estop = false;

        const uint32_t now_us_val = now_us();
        const uint32_t delta_us = now_us_val - last_motor_us;
        last_motor_us = now_us_val;

        const float dt_s = static_cast<float>(delta_us) * 1.0e-6f;

        float v_target = 0.0f;
        float w_target = 0.0f;
        if (safety.state() == diff_drive_safety::RUN && !safety.timeout_active()) {
          v_target = cmd_v_mps;
          w_target = cmd_w_radps;
        }

        last_targets = controller.update(v_target, w_target, dt_s);

        const bool enable = (safety.state() == diff_drive_safety::RUN) &&
                            !safety.timeout_active() &&
                            motor.valid();
        motor.set_wheels_mps(last_targets.vL_mps, last_targets.vR_mps, enable);

        uint32_t flags = safety.fault_flags();
        if (last_targets.saturated) {
          flags |= diff_drive_safety::FAULT_FLAG_SATURATED;
        }
        if (last_targets.ramp_limited) {
          flags |= diff_drive_safety::FAULT_FLAG_RAMP_LIMITED;
        }

        std::cout << "[OUT] applied_v=" << last_targets.applied_v_mps
                  << " applied_w=" << last_targets.applied_w_radps
                  << " vL=" << last_targets.vL_mps
                  << " vR=" << last_targets.vR_mps
                  << " flags=" << flags_to_string(flags) << "\n";

        diff_drive_status::StatusInputs status{};
        status.seq_ack = last_cmd_seq;
        status.applied_v_mps = last_targets.applied_v_mps;
        status.applied_w_radps = last_targets.applied_w_radps;
        status.target_vL_mps = last_targets.vL_mps;
        status.target_vR_mps = last_targets.vR_mps;
        status.fault_flags = flags;
        status.cmd_age_ms = cmd_age_ms_after;

        const diff_drive_protocol::ProtocolCounters &counters = diff_drive_protocol::protocol_counters();
        status.crc_fail_count = counters.crc_fail_count;
        status.resync_count = counters.resync_count;
        status.unknown_count = counters.unknown_count;
        status.seq_drop_count = counters.seq_drop_count;
        status.timeout_count = safety.timeout_count();
        print_status_readable(status);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      processed.insert(name);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}
#else
void setup() {
  Serial.begin(SERIAL_BAUD);

  motor.begin();
  controller.reset();
  safety.reset();

  if (DRIVER_FAULT_PIN >= 0) {
    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);
  }

  const uint32_t now_ms = millis();
  last_valid_cmd_ms = now_ms;
  last_status_ms = now_ms;
  last_motor_us = micros();
}

void loop() {
  const uint32_t now_ms = millis();

  while (diff_drive_protocol::protocol_read_frame(Serial, rx_frame)) {
    switch (rx_frame.msg_type) {
      case diff_drive_protocol::CMD_VEL: {
        const float v = read_f32_le(&rx_frame.payload[0]);
        const float w = read_f32_le(&rx_frame.payload[4]);
        cmd_v_mps = v;
        cmd_w_radps = w;
        last_valid_cmd_ms = now_ms;
        last_cmd_seq = rx_frame.seq;
        cmd_vel_received = true;
        break;
      }
      case diff_drive_protocol::CMD_ESTOP:
        cmd_estop = true;
        break;
      case diff_drive_protocol::CMD_CLEAR_ESTOP:
        cmd_clear_estop = true;
        break;
      default:
        break;
    }
  }

  const uint32_t age_ms = now_ms - last_valid_cmd_ms;
  const uint16_t cmd_age_ms = clamp_age_ms(age_ms);

  bool fault_pin_active = false;
  if (DRIVER_FAULT_PIN >= 0) {
    fault_pin_active = (digitalRead(DRIVER_FAULT_PIN) == LOW);
  }

  safety.update(cmd_age_ms, cmd_vel_received, cmd_estop, cmd_clear_estop, fault_pin_active);

  cmd_vel_received = false;
  cmd_estop = false;
  cmd_clear_estop = false;

  const uint32_t now_us = micros();
  if (static_cast<uint32_t>(now_us - last_motor_us) >= 1000u) {
    const uint32_t delta_us = static_cast<uint32_t>(now_us - last_motor_us);
    last_motor_us = now_us;

    const float dt_s = static_cast<float>(delta_us) * 1.0e-6f;

    float v_target = 0.0f;
    float w_target = 0.0f;
    if (safety.state() == diff_drive_safety::RUN && !safety.timeout_active()) {
      v_target = cmd_v_mps;
      w_target = cmd_w_radps;
    }

    last_targets = controller.update(v_target, w_target, dt_s);

    const bool enable = (safety.state() == diff_drive_safety::RUN) &&
                        !safety.timeout_active() &&
                        motor.valid();
    motor.set_wheels_mps(last_targets.vL_mps, last_targets.vR_mps, enable);
  }

  const uint32_t status_interval_ms = 1000u / STATUS_RATE_HZ;
  if (static_cast<uint32_t>(now_ms - last_status_ms) >= status_interval_ms) {
    last_status_ms = now_ms;

    diff_drive_status::StatusInputs status{};
    status.seq_ack = last_cmd_seq;
    status.applied_v_mps = last_targets.applied_v_mps;
    status.applied_w_radps = last_targets.applied_w_radps;
    status.target_vL_mps = last_targets.vL_mps;
    status.target_vR_mps = last_targets.vR_mps;
    status.fault_flags = safety.fault_flags();
    if (last_targets.saturated) {
      status.fault_flags |= diff_drive_safety::FAULT_FLAG_SATURATED;
    }
    if (last_targets.ramp_limited) {
      status.fault_flags |= diff_drive_safety::FAULT_FLAG_RAMP_LIMITED;
    }
    status.cmd_age_ms = cmd_age_ms;

    const diff_drive_protocol::ProtocolCounters &counters = diff_drive_protocol::protocol_counters();
    status.crc_fail_count = counters.crc_fail_count;
    status.resync_count = counters.resync_count;
    status.unknown_count = counters.unknown_count;
    status.seq_drop_count = counters.seq_drop_count;
    status.timeout_count = safety.timeout_count();

    uint8_t payload[34];
    diff_drive_status::build_status_payload(payload, status);
    diff_drive_protocol::protocol_write_frame(Serial, diff_drive_protocol::STATUS, status_seq++, payload, 34);
  }
}
#endif
