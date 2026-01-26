#include "status.h"

#include <string.h>

namespace diff_drive_status {

namespace {
void write_u16_le(uint8_t *buf, uint16_t value) {
  buf[0] = static_cast<uint8_t>(value & 0xFFu);
  buf[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void write_u32_le(uint8_t *buf, uint32_t value) {
  buf[0] = static_cast<uint8_t>(value & 0xFFu);
  buf[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  buf[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  buf[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

void write_f32_le(uint8_t *buf, float value) {
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  uint8_t temp[4];
  memcpy(temp, &value, 4);
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  buf[0] = temp[3];
  buf[1] = temp[2];
  buf[2] = temp[1];
  buf[3] = temp[0];
#else
  buf[0] = temp[0];
  buf[1] = temp[1];
  buf[2] = temp[2];
  buf[3] = temp[3];
#endif
}
} // namespace

void build_status_payload(uint8_t *out_payload_34, const StatusInputs &inputs) {
  uint8_t *p = out_payload_34;

  write_u16_le(p, inputs.seq_ack);
  p += 2;

  write_f32_le(p, inputs.applied_v_mps);
  p += 4;
  write_f32_le(p, inputs.applied_w_radps);
  p += 4;
  write_f32_le(p, inputs.target_vL_mps);
  p += 4;
  write_f32_le(p, inputs.target_vR_mps);
  p += 4;

  write_u32_le(p, inputs.fault_flags);
  p += 4;

  write_u16_le(p, inputs.cmd_age_ms);
  p += 2;

  write_u16_le(p, inputs.crc_fail_count);
  p += 2;
  write_u16_le(p, inputs.resync_count);
  p += 2;
  write_u16_le(p, inputs.unknown_count);
  p += 2;
  write_u16_le(p, inputs.seq_drop_count);
  p += 2;
  write_u16_le(p, inputs.timeout_count);
  p += 2;
}

} // namespace diff_drive_status
