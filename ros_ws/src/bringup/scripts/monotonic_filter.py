"""Drop messages whose stamp does not move forward. Shared by the sim relays.

The AutoDRIVE bridge occasionally publishes a message stamped slightly EARLIER
than the one before it on the same topic - seen live as ~0.2 ms backwards, on
the lidar and the IMU in the same tick, roughly once every few tens of seconds.
Probably two bridge ticks being handled concurrently, each stamping with now().

Cartographer requires every sensor's stamps to strictly increase, and aborts the
whole node on the first one that does not:
    ordered_multi_queue.cc: Check failed: last_dispatched_time_ <= ...
        Non-sorted data added to queue: '(0, imu)'
    sensor_bridge.cpp: Ignored subdivision ... is not before current ...

Dropping the odd sample costs one message in a few thousand at ~57 Hz.
Rewriting its stamp instead would put sensor data at a time it was not taken.
"""
from rclpy.node import Node


class MonotonicFilter:
    def __init__(self, node: Node, label: str):
        self._node = node
        self._label = label
        self._last_ns = None
        self._dropped = 0

    def accept(self, stamp) -> bool:
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self._last_ns is not None and ns <= self._last_ns:
            self._dropped += 1
            self._node.get_logger().warn(
                f"dropped {self._label} stamped {(self._last_ns - ns) / 1e6:.3f} ms "
                f"before the previous one ({self._dropped} so far)",
                throttle_duration_sec=5.0)
            return False
        self._last_ns = ns
        return True
