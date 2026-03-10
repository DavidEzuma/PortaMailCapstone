#!/usr/bin/env python3
"""Pre-spin the RPLIDAR motor via DTR, then disable HUPCL so the motor
stays at operating speed when the ROS driver opens the port.

Usage: prespinlidar.py [serial_port] [warmup_seconds]
"""
import fcntl
import os
import struct
import sys
import termios
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

# IOCTL constants (Linux ARM/x86)
TIOCMBIS = 0x5416   # set modem bits
TIOCM_DTR = 0x002   # DTR line = MOTOCTL on RPLIDAR adapter

print(f'[prespinlidar] Opening {port} ...', flush=True)
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

# Disable HUPCL so DTR stays asserted after we close the fd
attrs = termios.tcgetattr(fd)
attrs[2] &= ~termios.HUPCL
termios.tcsetattr(fd, termios.TCSANOW, attrs)

# Assert DTR → motor starts
fcntl.ioctl(fd, TIOCMBIS, struct.pack('I', TIOCM_DTR))

print(f'[prespinlidar] Motor spinning — waiting {duration:.0f}s for stabilisation ...', flush=True)
time.sleep(duration)

os.close(fd)  # DTR stays high (HUPCL disabled)
print('[prespinlidar] Motor warmed up. Handing off to ROS driver.', flush=True)
