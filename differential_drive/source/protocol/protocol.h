#pragma once

#include "../config/build_mode.h"

#if DIFF_DRIVE_DEBUG_SW
#include <cstddef>
#include <cstdint>
class Stream {
public:
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t write(uint8_t byte) = 0;
  virtual size_t write(const uint8_t *buffer, size_t size) = 0;
};
#else
#include <Arduino.h>
#include <stdint.h>
#endif

namespace diff_drive_protocol {

constexpr uint8_t PROTO_SOF_1 = 0xAA;
constexpr uint8_t PROTO_SOF_2 = 0x55;
constexpr uint8_t PROTO_VERSION = 0x01;

constexpr uint16_t PROTO_MAX_PAYLOAD = 64;
constexpr uint8_t PROTO_HEADER_LEN = 2 + 1 + 1 + 1 + 2;
constexpr uint16_t PROTO_MAX_FRAME = PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + 2;

enum MsgType : uint8_t {
  CMD_VEL = 0x01,
  CMD_ESTOP = 0x02,
  CMD_CLEAR_ESTOP = 0x03,
  STATUS = 0x10
};

struct Frame {
  uint8_t msg_type;
  uint16_t seq;
  uint8_t payload_len;
  uint8_t payload[PROTO_MAX_PAYLOAD];
};

struct ProtocolCounters {
  uint16_t crc_fail_count;
  uint16_t resync_count;
  uint16_t unknown_count;
  uint16_t seq_drop_count;
};

bool protocol_read_frame(Stream &stream, Frame &out_frame);
bool protocol_write_frame(Stream &stream, uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint8_t len);
const ProtocolCounters &protocol_counters();

} // namespace diff_drive_protocol
