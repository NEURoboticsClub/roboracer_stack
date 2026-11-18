#!/usr/bin/env python3
"""Republish the AutoDRIVE sim IMU with gravity added to linear_acceleration.

WHY. A real accelerometer measures gravity: at rest it reads about +9.81 m/s^2
straight up, in its own frame. The AutoDRIVE sim does not - measured at rest,
its linear_acceleration is ~0 on every axis (z ~0.002). Cartographer works out
which way is up from that reading and aborts (an ImuTracker CHECK failure) or
mis-levels every scan when it is ~0, so it cannot take the sim IMU as is.

WHAT. Rotate the world's gravity (0, 0, +g) into the IMU frame using the
message's own roll and pitch, and add it:

    linear_acceleration += R(q)^T * (0, 0, g)

which is what a real accelerometer would read in that attitude. Yaw does not
enter (gravity points straight down whatever the heading), so none of the
sim's ground-truth heading leaks through here. Everything else in the message
is passed through untouched; Cartographer ignores the orientation field anyway.

ALSO drops any sample stamped at or before the last one passed on: the bridge
occasionally goes ~0.2 ms backwards and Cartographer aborts on it. See
monotonic_filter.py.

    in   /autodrive/roboracer_1/imu                sensor_msgs/Imu
    out  /autodrive/roboracer_1/imu_with_gravity   sensor_msgs/Imu

ASSUMES the sim's linear_acceleration is in the IMU (body) frame, like a real
IMU. That cannot be told apart from a world-frame reading while the car is
stopped; to check, accelerate straight ahead and confirm the bump shows on x
(the car's forward axis), not split between x and y by the heading.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from monotonic_filter import MonotonicFilter

GRAVITY = 9.80665  # m/s^2


class ImuGravityRelay(Node):
    def __init__(self):
        super().__init__("imu_gravity_relay")
        self.gravity = self.declare_parameter("gravity", GRAVITY).value
        # The bridge publishes RELIABLE at depth 1; a deeper queue here just
        # avoids dropping a sample while this callback runs.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Imu, "imu_out", qos)
        self.sub = self.create_subscription(Imu, "imu_in", self.callback, qos)
        self.monotonic = MonotonicFilter(self, "imu")

    def callback(self, msg: Imu):
        if not self.monotonic.accept(msg.header.stamp):
            return
        q = msg.orientation
        g = self.gravity
        # Third row of the rotation matrix of q (IMU -> world), i.e. world +z
        # expressed in the IMU frame.
        msg.linear_acceleration.x += g * 2.0 * (q.x * q.z - q.w * q.y)
        msg.linear_acceleration.y += g * 2.0 * (q.y * q.z + q.w * q.x)
        msg.linear_acceleration.z += g * (1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuGravityRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
