import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time
import sys

# The final, correct path format for your file in WSL:
LOG_FILE_PATH = r"/mnt/c/Users/David/OneDrive/Documents/Projects/PortaMailCapstone/Datasets/mit-killian.txt"

class CarmenPlayer(Node):
    def __init__(self):
        super().__init__('carmen_player')
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to read the file line by line
        self.timer = self.create_timer(0.1, self.read_and_publish) # 10Hz playback
        
        try:
            self.f = open(LOG_FILE_PATH, 'r')
            self.get_logger().info(f"Opened log file: {LOG_FILE_PATH}")
        except FileNotFoundError:
            self.get_logger().error(f"Could not find file at: {LOG_FILE_PATH}")
            exit(1)

    def read_and_publish(self):
        line = self.f.readline()
        if not line:
            self.get_logger().info("End of file reached")
            self.timer.cancel()
            return

        parts = line.split()
        if not parts:
            return

        # Handle FLASER (Laser Scans)
        if parts[0] == 'FLASER':
            num_readings = int(parts[1])
            # Parse ranges (FLASER format: type count r1 r2 ... rn x y theta ...)
            ranges = [float(x) for x in parts[2:2+num_readings]]
            
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = 'base_link'
            
            # SICK LMS 200 specs (typical for MIT datasets)
            scan.angle_min = -1.5708 # -90 degrees
            scan.angle_max = 1.5708  # +90 degrees
            scan.angle_increment = 3.14159 / num_readings
            scan.range_min = 0.0
            scan.range_max = 80.0
            scan.ranges = ranges
            
            self.scan_pub.publish(scan)

        # Handle ODOM (Odometry)
        elif parts[0] == 'ODOM':
            # ODOM x y theta
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])

            current_time = self.get_clock().now().to_msg()

            # 1. Publish TF (Required for RViz to visualize movement)
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            
            # Convert theta (yaw) to Quaternion
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            
            self.tf_broadcaster.sendTransform(t)

            # 2. Publish Odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation = t.transform.rotation
            
            self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = CarmenPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()