#pragma once

#include <stdint.h>
#include <stddef.h>

namespace diff_drive_protocol {

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

} // namespace diff_drive_protocol
