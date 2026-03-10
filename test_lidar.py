import time
from adafruit_rplidar import RPLidar, RPLidarException

# Initialize the LIDAR (None is required here for the motor pin argument)
lidar = RPLidar(None, '/dev/ttyUSB0', baudrate=256000)

print("Connecting to RPLidar...")
try:
    # Use properties instead of methods for the Adafruit library
    print("Device Info:", lidar.info)
    print("Health Status:", lidar.health)
    
    print("\nSpinning up motor... waiting for it to stabilize.")
    lidar.start_motor()
    time.sleep(3) # Crucial: Let the motor reach full speed
    
    print("Clearing serial buffer...")
    lidar.clear_input() # Discard any junk data generated during startup
    
    print("Reading scans (Press Ctrl+C to stop)...")
    for i, scan in enumerate(lidar.iter_scans()):
        print(f"\n--- 360° Scan {i+1} ---")
        print(f"Total points detected: {len(scan)}")
        
        for point in scan[:3]:
            # Adafruit's library returns (quality, angle, distance)
            quality, angle, distance = point
            print(f"Angle: {angle:.2f}°, Distance: {distance:.2f} mm, Signal Quality: {quality}")

        if i >= 4:
            break

except RPLidarException as e:
    print(f"\nLidar Error: {e}")
except KeyboardInterrupt:
    print("\nStopping manually...")
finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Disconnected safely.")
