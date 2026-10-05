#!/usr/bin/env python3
"""LiDAR heartbeat for lidar_watchdog.sh (runs in the container).

Subscribes to /scan (BEST_EFFORT, like every other reader) and once per
second writes "<now> <time of the last scan>" (epoch seconds) to
/workspace/.lidar_alive. The watchdog on the Jetson reads it: the first
number says this script is alive, the second whether scans arrive.

Why a file and not ros2 topic hz from the host: that starts a whole ROS CLI
every few seconds (1-2 s of CPU each time). This costs nothing.
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

OUT = '/workspace/.lidar_alive'


class LidarAlive(Node):
    def __init__(self):
        super().__init__('lidar_alive')
        self.last_scan = 0.0
        self.create_subscription(LaserScan, '/scan', self._scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(1.0, self._write)

    def _scan(self, _msg):
        self.last_scan = time.time()

    def _write(self):
        tmp = OUT + '.tmp'
        with open(tmp, 'w') as f:
            f.write('%.1f %.1f\n' % (time.time(), self.last_scan))
        os.replace(tmp, OUT)


def main():
    rclpy.init()
    node = LidarAlive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
