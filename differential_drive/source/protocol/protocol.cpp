#include "protocol.h"

#include "crc16_ccitt.h"

namespace diff_drive_protocol {

namespace {
struct ParserState {
  enum State : uint8_t {
    WAIT_SOF_1 = 0,
    WAIT_SOF_2 = 1,
    READ_VERSION = 2,
    READ_MSG_TYPE = 3,
    READ_LEN = 4,
    READ_SEQ_L = 5,
    READ_SEQ_H = 6,
    READ_PAYLOAD = 7,
    READ_CRC_L = 8,
    READ_CRC_H = 9
  } state;

  Frame frame;
  uint8_t version;
  uint8_t payload_index;
  uint16_t crc_rx;
  uint16_t frame_bytes;
};

ParserState parser{ParserState::WAIT_SOF_1, {}, 0, 0, 0, 0};
ProtocolCounters counters{0, 0, 0, 0};
uint16_t last_seq = 0;
bool have_last_seq = false;

uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

bool payload_len_ok(uint8_t msg_type, uint8_t payload_len) {
  switch (msg_type) {
    case CMD_VEL:
      return payload_len == 8;
    case CMD_ESTOP:
      return payload_len == 0;
    case CMD_CLEAR_ESTOP:
      return payload_len == 0;
    case STATUS:
      return payload_len == 34;
    default:
      return false;
  }
}

bool is_defined_type(uint8_t msg_type) {
  return (msg_type == CMD_VEL) || (msg_type == CMD_ESTOP) || (msg_type == CMD_CLEAR_ESTOP) ||
         (msg_type == STATUS);
}

bool is_rx_allowed_type(uint8_t msg_type) {
  return (msg_type == CMD_VEL) || (msg_type == CMD_ESTOP) || (msg_type == CMD_CLEAR_ESTOP);
}

void reset_to_sof() {
  parser.state = ParserState::WAIT_SOF_1;
  parser.payload_index = 0;
  parser.crc_rx = 0;
  parser.frame_bytes = 0;
}

} // namespace

const ProtocolCounters &protocol_counters() {
  return counters;
}

bool protocol_read_frame(Stream &stream, Frame &out_frame) {
  while (stream.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(stream.read());

    if (parser.state != ParserState::WAIT_SOF_1) {
      parser.frame_bytes++;
      if (parser.frame_bytes > PROTO_MAX_FRAME) {
        counters.resync_count++;
        reset_to_sof();
        continue;
      }
    }

    switch (parser.state) {
      case ParserState::WAIT_SOF_1:
        if (byte == PROTO_SOF_1) {
          parser.state = ParserState::WAIT_SOF_2;
          parser.frame_bytes = 1;
        }
        break;

      case ParserState::WAIT_SOF_2:
        if (byte == PROTO_SOF_2) {
          parser.state = ParserState::READ_VERSION;
        } else {
          reset_to_sof();
        }
        break;

      case ParserState::READ_VERSION:
        parser.version = byte;
        if (parser.version != PROTO_VERSION) {
          counters.resync_count++;
          reset_to_sof();
        } else {
          parser.state = ParserState::READ_MSG_TYPE;
        }
        break;

      case ParserState::READ_MSG_TYPE:
        parser.frame.msg_type = byte;
        parser.state = ParserState::READ_LEN;
        break;

      case ParserState::READ_LEN:
        parser.frame.payload_len = byte;
        if (parser.frame.payload_len > PROTO_MAX_PAYLOAD) {
          counters.resync_count++;
          reset_to_sof();
        } else {
          parser.state = ParserState::READ_SEQ_L;
        }
        break;

      case ParserState::READ_SEQ_L:
        parser.frame.seq = static_cast<uint16_t>(byte);
        parser.state = ParserState::READ_SEQ_H;
        break;

      case ParserState::READ_SEQ_H:
        parser.frame.seq |= static_cast<uint16_t>(byte) << 8;
        parser.payload_index = 0;
        if (parser.frame.payload_len == 0) {
          parser.state = ParserState::READ_CRC_L;
        } else {
          parser.state = ParserState::READ_PAYLOAD;
        }
        break;

      case ParserState::READ_PAYLOAD:
        if (parser.payload_index < parser.frame.payload_len) {
          parser.frame.payload[parser.payload_index++] = byte;
          if (parser.payload_index >= parser.frame.payload_len) {
            parser.state = ParserState::READ_CRC_L;
          }
        } else {
          counters.resync_count++;
          reset_to_sof();
        }
        break;

      case ParserState::READ_CRC_L:
        parser.crc_rx = static_cast<uint16_t>(byte);
        parser.state = ParserState::READ_CRC_H;
        break;

      case ParserState::READ_CRC_H: {
        parser.crc_rx |= static_cast<uint16_t>(byte) << 8;

        uint8_t header[5];
        header[0] = parser.version;
        header[1] = parser.frame.msg_type;
        header[2] = parser.frame.payload_len;
        header[3] = static_cast<uint8_t>(parser.frame.seq & 0xFFu);
        header[4] = static_cast<uint8_t>((parser.frame.seq >> 8) & 0xFFu);

        uint16_t crc_calc = 0xFFFFu;
        crc_calc = crc16_update(crc_calc, header, sizeof(header));
        if (parser.frame.payload_len > 0) {
          crc_calc = crc16_update(crc_calc, parser.frame.payload, parser.frame.payload_len);
        }

        if (crc_calc != parser.crc_rx) {
          counters.crc_fail_count++;
          counters.resync_count++;
          reset_to_sof();
          break;
        }

        if (parser.frame.msg_type == STATUS) {
          counters.resync_count++;
          reset_to_sof();
          break;
        }

        if (!is_rx_allowed_type(parser.frame.msg_type)) {
          if (!is_defined_type(parser.frame.msg_type)) {
            counters.unknown_count++;
          }
          counters.resync_count++;
          reset_to_sof();
          break;
        }

        if (!payload_len_ok(parser.frame.msg_type, parser.frame.payload_len)) {
          counters.resync_count++;
          reset_to_sof();
          break;
        }

        if (parser.frame.msg_type == CMD_VEL) {
          if (have_last_seq) {
            const uint16_t expected = static_cast<uint16_t>(last_seq + 1u);
            if (parser.frame.seq != expected) {
              counters.seq_drop_count++;
            }
          }
          last_seq = parser.frame.seq;
          have_last_seq = true;
        }

        out_frame = parser.frame;
        reset_to_sof();
        return true;
      }
    }
  }

  return false;
}

bool protocol_write_frame(Stream &stream, uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint8_t len) {
  if (len > PROTO_MAX_PAYLOAD) {
    return false;
  }

  uint8_t header[5];
  header[0] = PROTO_VERSION;
  header[1] = msg_type;
  header[2] = len;
  header[3] = static_cast<uint8_t>(seq & 0xFFu);
  header[4] = static_cast<uint8_t>((seq >> 8) & 0xFFu);

  uint16_t crc = 0xFFFFu;
  crc = crc16_update(crc, header, sizeof(header));
  if (len > 0 && payload != nullptr) {
    crc = crc16_update(crc, payload, len);
  }

  stream.write(PROTO_SOF_1);
  stream.write(PROTO_SOF_2);
  stream.write(header, sizeof(header));
  if (len > 0 && payload != nullptr) {
    stream.write(payload, len);
  }
  stream.write(static_cast<uint8_t>(crc & 0xFFu));
  stream.write(static_cast<uint8_t>((crc >> 8) & 0xFFu));

  return true;
}

} // namespace diff_drive_protocol
