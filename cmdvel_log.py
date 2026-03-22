#!/usr/bin/env python3
"""
cmdvel_log.py — logs /cmd_vel with timestamps and flags unexpected non-zero
commands. Press Enter in the terminal to mark "joystick untouched" moments.
Ctrl+C to stop and save log to cmdvel_log.csv.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 ~/Documents/PortaMailCapstone/cmdvel_log.py
"""
import rclpy, threading, time, csv, sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

LOG_FILE = '/home/david-ezuma/Documents/PortaMailCapstone/cmdvel_log.csv'

class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmdvel_logger')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.records = []
        self.marker = 'driving'
        self.get_logger().info('Logging /cmd_vel. Press Enter to mark "UNTOUCHED". Ctrl+C to finish.')

    def cb(self, msg):
        now = time.monotonic()
        linear  = round(msg.linear.x, 4)
        angular = round(msg.angular.z, 4)
        nonzero = abs(linear) > 0.001 or abs(angular) > 0.001
        flag = '*** NONZERO UNTOUCHED ***' if (self.marker == 'UNTOUCHED' and nonzero) else ''
        record = [f'{now:.4f}', linear, angular, self.marker, flag]
        self.records.append(record)
        if flag:
            print(f'[{now:.3f}] !! Non-zero while UNTOUCHED: linear={linear}  angular={angular}')

def input_thread(node):
    while True:
        input()
        node.marker = 'UNTOUCHED'
        print('>> Marked: UNTOUCHED')
        time.sleep(2.0)
        node.marker = 'driving'
        print('>> Marker cleared')

def main():
    rclpy.init()
    node = CmdVelLogger()
    t = threading.Thread(target=input_thread, args=(node,), daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    with open(LOG_FILE, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time_s', 'linear_x', 'angular_z', 'marker', 'flag'])
        w.writerows(node.records)

    nonzero_untouched = [r for r in node.records if r[4]]
    print(f'\n--- Summary ---')
    print(f'Total messages   : {len(node.records)}')
    print(f'Nonzero UNTOUCHED: {len(nonzero_untouched)}')
    if nonzero_untouched:
        print('Sample values:')
        for r in nonzero_untouched[:10]:
            print(f'  linear={r[1]}  angular={r[2]}')
    print(f'Full log saved to: {LOG_FILE}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
