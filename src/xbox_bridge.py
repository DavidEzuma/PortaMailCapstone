import pygame
import socket
import time
import struct

# --- CONFIGURATION ---
WSL_IP = "127.0.0.1" # Localhost usually works for Win->WSL
PORT = 5005
# ---------------------

# Initialize Controller
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found! Connect via Bluetooth first.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Setup UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Listening to {joystick.get_name()}...")
print(f"Sending data to {WSL_IP}:{PORT}")
print("Press Ctrl+C to stop.")

try:
    while True:
        pygame.event.pump()
        
        # Read Axes (Adjust index 0-3 based on your specific controller mapping)
        # Usually 0=LeftStickX, 1=LeftStickY, 3=RightStickX, 4=RightStickY
        linear_x = -joystick.get_axis(1)  # Invert Y for standard forward/back
        angular_z = -joystick.get_axis(0) # Invert X for standard left/right

        # Apply a deadzone to stop drift
        if abs(linear_x) < 0.1: linear_x = 0.0
        if abs(angular_z) < 0.1: angular_z = 0.0

        # Pack data: 'ff' means two floats (4 bytes each)
        data = struct.pack('ff', linear_x, angular_z)
        
        sock.sendto(data, (WSL_IP, PORT))
        
        time.sleep(0.05) # 20Hz refresh rate

except KeyboardInterrupt:
    print("\nClosing bridge.")