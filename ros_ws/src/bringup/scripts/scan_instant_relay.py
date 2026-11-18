#!/usr/bin/env python3
"""Republish the AutoDRIVE sim lidar as an instantaneous scan, for Cartographer.

WHY. The bridge labels every scan as a 25 ms sweep (scan_time = 1/40 s,
time_increment = scan_time / 1080), but it publishes one every ~17 ms (~56 Hz,
measured). Cartographer believes the label: it timestamps each beam at
header.stamp + i * time_increment, so every scan ENDS 25 ms after its stamp,
after the NEXT scan has already started. Consecutive scans overlap in time,
which shows up as a flood of
    range_data_collator.cc: Dropped N earlier points
    sensor_bridge.cpp: Ignored subdivision ... is not before current
and, with the IMU on, an IMU sample stamped before the end of the last scan
aborts the node:
    pose_extrapolator.cc: Check failed: ... imu_data.time >= timed_pose_queue_.back().time

The label is also simply wrong for this sensor. Unity raycasts all 1080 beams in
the same physics tick, so there is no sweep and nothing to de-skew.

WHAT. Copy the scan with time_increment = 0, so every beam is at header.stamp,
which matches how the sim actually takes it. Nothing else changes. scan_time is
left as the bridge set it; Cartographer does not use it.

ALSO drops any scan stamped at or before the last one passed on. The bridge
occasionally publishes a message stamped ~0.2 ms EARLIER than the previous one
(seen live; probably two bridge ticks handled concurrently), and Cartographer
aborts on the first one ("Non-sorted data added to queue"). See
monotonic_filter.py.

    in   /autodrive/roboracer_1/lidar           sensor_msgs/LaserScan
    out  /autodrive/roboracer_1/lidar_instant   sensor_msgs/LaserScan

Only Cartographer reads the output. The planner keeps the bridge's topic.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

from monotonic_filter import MonotonicFilter


class ScanInstantRelay(Node):
    def __init__(self):
        super().__init__("scan_instant_relay")
        # The bridge publishes RELIABLE at depth 1.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(LaserScan, "scan_out", qos)
        self.sub = self.create_subscription(LaserScan, "scan_in", self.callback, qos)
        self.monotonic = MonotonicFilter(self, "scan")

    def callback(self, msg: LaserScan):
        if not self.monotonic.accept(msg.header.stamp):
            return
        msg.time_increment = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanInstantRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
