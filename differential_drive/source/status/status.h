#pragma once

#include <stdint.h>

namespace diff_drive_status {

struct StatusInputs {
  uint16_t seq_ack;
  float applied_v_mps;
  float applied_w_radps;
  float target_vL_mps;
  float target_vR_mps;
  uint32_t fault_flags;
  uint16_t cmd_age_ms;
  uint16_t crc_fail_count;
  uint16_t resync_count;
  uint16_t unknown_count;
  uint16_t seq_drop_count;
  uint16_t timeout_count;
};

void build_status_payload(uint8_t *out_payload_34, const StatusInputs &inputs);

} // namespace diff_drive_status
