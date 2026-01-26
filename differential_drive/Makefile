CPP := clang++
CXXFLAGS := -std=c++17 -Wall -Wextra -DDIFF_DRIVE_DEBUG_SW

SRC := \
	source/main.cpp \
	source/control/diff_drive.cpp \
	source/control/ramp.cpp \
	source/safety/safety.cpp \
	source/status/status.cpp \
	source/protocol/protocol.cpp \
	source/protocol/crc16_ccitt.cpp \
	source/drivers/motor_driver.cpp

BIN := /tmp/diff_drive_debug

.PHONY: debug_sw clean

debug_sw:
	$(CPP) $(CXXFLAGS) $(SRC) -o $(BIN)

clean:
	rm -f $(BIN)
