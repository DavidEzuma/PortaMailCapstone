#!/usr/bin/env python3
"""
jitter_test.py — measures Bluetooth controller jitter on /joy topic.
Run for ~30 seconds while moving the stick around, then Ctrl+C for a summary.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 jitter_test.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import statistics, time

class JitterTest(Node):
    def __init__(self):
        super().__init__('jitter_test')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self.last_time = None
        self.intervals = []
        self.gaps = []   # intervals > 100ms (likely dropped packets)
        self.get_logger().info('Listening on /joy — move the stick and hold RB. Ctrl+C for summary.')

    def cb(self, msg):
        now = time.monotonic()
        if self.last_time is not None:
            dt_ms = (now - self.last_time) * 1000.0
            self.intervals.append(dt_ms)
            if dt_ms > 100.0:
                self.gaps.append(dt_ms)
                self.get_logger().warn(f'GAP detected: {dt_ms:.1f} ms')
        self.last_time = now

def main():
    rclpy.init()
    node = JitterTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    intervals = node.intervals
    if len(intervals) < 10:
        print('\nNot enough data — run longer and move the stick.')
        return

    expected_ms = 1000.0 / 50.0  # autorepeat_rate = 50 Hz → 20ms expected
    print(f'\n--- Bluetooth Jitter Report ({len(intervals)} samples) ---')
    print(f'Expected interval : {expected_ms:.1f} ms  (50 Hz autorepeat)')
    print(f'Mean interval     : {statistics.mean(intervals):.1f} ms')
    print(f'Std deviation     : {statistics.stdev(intervals):.1f} ms  (jitter)')
    print(f'Min               : {min(intervals):.1f} ms')
    print(f'Max               : {max(intervals):.1f} ms')
    print(f'Gaps >100ms       : {len(node.gaps)}  {[f"{g:.0f}ms" for g in node.gaps]}')
    print()
    if statistics.stdev(intervals) < 5:
        print('Verdict: LOW jitter — Bluetooth is stable.')
    elif statistics.stdev(intervals) < 15:
        print('Verdict: MODERATE jitter — occasional delays but manageable.')
    else:
        print('Verdict: HIGH jitter — Bluetooth is likely causing the erratic motor behavior.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
