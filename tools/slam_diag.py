#!/usr/bin/env python3
"""Diagnose systematic heading error in the mapping stack.

Run it with the mapping launch already up, then drive a full lap:

    python3 tools/slam_diag.py --seconds 120

It answers, in order, the only questions that distinguish the remaining causes
of a map that rotates a fixed amount every lap.

  1. Is the scan's angular calibration self-consistent?
     A reported angle_increment that disagrees with (angle_max-angle_min)/(N-1)
     stretches every scan angularly. Straight walls then render as arcs, and
     matching arc to arc biases rotation the same way every time.

  2. Do straight walls come out straight?
     Fits a line to the longest run of returns and reports the worst deviation.
     A real wall bowing by more than a couple of centimetres over its length is
     the angular calibration above, seen directly.

  3. Where does the heading error live - odometry, or SLAM?
     Tracks yaw in both frames at once:
       odom -> base_link  = raw odometry, uncorrected
       map  -> base_link  = what SLAM believes after scan matching
     Drive a closed lap and both should come back to where they started. If only
     odometry drifts, the map is fine and scan matching is doing its job. If BOTH
     drift by the same amount, scan matching is inheriting the error rather than
     correcting it, and no amount of odometry calibration will help.

  4. Is loop closure ever firing?
     map -> odom IS the accumulated correction, so it only moves when SLAM
     revises its estimate. A loop closure shows up as a step change. No steps
     over a full lap means the lap never closed, which is what leaves one
     rotated copy of the track per lap instead of one track.
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(prev, cur):
    """Shortest signed step from prev to cur, so we can accumulate past +-pi."""
    return (cur - prev + math.pi) % (2.0 * math.pi) - math.pi


class Diag(Node):
    def __init__(self):
        super().__init__('slam_diag')
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.scan = None
        self.odom_yaw = self.map_yaw = None
        self.odom_acc = self.map_acc = 0.0
        self.corr_prev = None
        self.jumps = []
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, m):
        if self.scan is None:
            self.scan = m

    def sample(self):
        for parent, child, attr in (('odom', 'base_link', 'odom'),
                                    ('map', 'base_link', 'map')):
            try:
                tf = self.buf.lookup_transform(parent, child, Time())
            except Exception:
                continue
            y = yaw_of(tf.transform.rotation)
            prev = getattr(self, f'{attr}_yaw')
            if prev is not None:
                setattr(self, f'{attr}_acc',
                        getattr(self, f'{attr}_acc') + unwrap(prev, y))
            setattr(self, f'{attr}_yaw', y)
        # map -> odom is the correction itself; steps in it are SLAM revising.
        try:
            tf = self.buf.lookup_transform('map', 'odom', Time())
        except Exception:
            return
        y = yaw_of(tf.transform.rotation)
        if self.corr_prev is not None:
            step = abs(unwrap(self.corr_prev, y))
            if step > math.radians(0.75):
                self.jumps.append(math.degrees(step))
        self.corr_prev = y


def check_scan(m):
    n = len(m.ranges)
    span = m.angle_max - m.angle_min
    implied = span / (n - 1) if n > 1 else float('nan')
    print("--- 1. scan angular calibration ---")
    print(f"  points              : {n}")
    print(f"  FOV                 : {math.degrees(span):.3f} deg  "
          f"[{math.degrees(m.angle_min):+.3f} .. {math.degrees(m.angle_max):+.3f}]")
    print(f"  angle_increment     : {math.degrees(m.angle_increment):.6f} deg  (reported)")
    print(f"  (angle_max-min)/N-1 : {math.degrees(implied):.6f} deg  (implied)")
    if implied and math.isfinite(implied):
        err = (m.angle_increment - implied) / implied * 100.0
        verdict = "OK" if abs(err) < 0.05 else "*** INCONSISTENT ***"
        print(f"  mismatch            : {err:+.4f} %   {verdict}")
        sweep = m.angle_min + (n - 1) * m.angle_increment
        print(f"  min + (N-1)*inc     : {math.degrees(sweep):+.3f} deg "
              f"(should equal angle_max {math.degrees(m.angle_max):+.3f})")
    print(f"  scan_time           : {m.scan_time:.5f} s"
          f"   -> {1.0/m.scan_time:.2f} Hz" if m.scan_time else "  scan_time: 0")
    print(f"  range_min/max       : {m.range_min} / {m.range_max}")
    zeros = [i for i, r in enumerate(m.ranges) if r == 0.0]
    nans = sum(1 for r in m.ranges if not math.isfinite(r))
    print(f"  zero returns        : {len(zeros)}/{n}"
          "   (SICK encodes no-return as 0.0)")
    print(f"  non-finite returns  : {nans}/{n}   (NaN/inf - correctly ignored downstream)")
    if m.range_min <= 0.0 and zeros:
        print("  *** range_min is 0.0, so those zeros are NOT filtered: they become")
        print("      valid points AT THE SENSOR ORIGIN. Enable the driver's range")
        print("      filter (range_filter_handling:=5, range_min:=0.05).")
        bearings = sorted(math.degrees(m.angle_min + i * m.angle_increment) for i in zeros)
        # group into contiguous clusters so a fixed occlusion is obvious
        groups, cur = [], [bearings[0]]
        for b in bearings[1:]:
            if b - cur[-1] <= 2.0:
                cur.append(b)
            else:
                groups.append(cur); cur = [b]
        groups.append(cur)
        print(f"      bearings: " + ", ".join(
            f"{g[0]:+.1f}..{g[-1]:+.1f} deg ({len(g)})" if len(g) > 1
            else f"{g[0]:+.1f} deg" for g in groups))
        print("      Clustered at fixed bearings => a permanent occlusion (mount,"
              " cable), so the bias it causes is systematic, not noise.")
    elif zeros:
        print(f"  range_min={m.range_min} filters them: OK")


def check_straightness(m, win_deg=30.0):
    """Flattest window in view -> is it actually flat?

    An earlier version took the longest contiguous run of returns, which in an
    enclosed room is most of the 270 deg field of view - it fitted one line
    through the whole perimeter and reported a nonsense "bow" of most of a metre.
    Instead, slide a modest window over the scan, fit each one, and report the
    BEST fit. The flattest 30 deg of an indoor scan is a wall; if even that bows,
    the geometry really is distorted.
    """
    pts = []
    for i, r in enumerate(m.ranges):
        if math.isfinite(r) and 0.2 < r < 15.0:
            a = m.angle_min + i * m.angle_increment
            pts.append((i, r * math.cos(a), r * math.sin(a)))

    print("\n--- 2. does a straight wall come out straight? ---")
    w = max(12, int(math.radians(win_deg) / m.angle_increment))
    if len(pts) < w:
        print(f"  fewer than {w} usable returns; nothing to fit")
        return

    best = None
    for s0 in range(0, len(pts) - w + 1):
        seg = pts[s0:s0 + w]
        # contiguous beams only, and no range step that would span two surfaces
        if seg[-1][0] - seg[0][0] != w - 1:
            continue
        if any(abs(math.hypot(seg[k+1][1]-seg[k][1], seg[k+1][2]-seg[k][2])) > 0.12
               for k in range(len(seg) - 1)):
            continue
        n = len(seg)
        mx = sum(q[1] for q in seg) / n
        my = sum(q[2] for q in seg) / n
        sxx = sum((q[1]-mx)**2 for q in seg)
        syy = sum((q[2]-my)**2 for q in seg)
        sxy = sum((q[1]-mx)*(q[2]-my) for q in seg)
        th = 0.5 * math.atan2(2.0*sxy, sxx - syy)
        nx, ny = -math.sin(th), math.cos(th)
        res = [abs((q[1]-mx)*nx + (q[2]-my)*ny) for q in seg]
        rms = math.sqrt(sum(r*r for r in res)/n)
        if best is None or rms < best[0]:
            length = math.hypot(seg[-1][1]-seg[0][1], seg[-1][2]-seg[0][2])
            best = (rms, max(res), length, n)

    if best is None:
        print("  no clean contiguous window found (cluttered scene?)")
        return
    rms, worst, length, n = best
    print(f"  flattest {win_deg:.0f} deg window: {n} points, {length:.2f} m long")
    print(f"  RMS deviation       : {rms*1000:.1f} mm")
    print(f"  worst deviation     : {worst*1000:.1f} mm")
    print("  verdict             : " + ("OK - geometry is undistorted" if worst < 0.03
          else "*** BOWED - scan geometry is distorted ***"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=120.0,
                    help="how long to watch; drive at least one full lap")
    args = ap.parse_args()

    rclpy.init()
    d = Diag()
    print("waiting for /scan ...")
    t0 = d.get_clock().now().nanoseconds
    while d.scan is None:
        rclpy.spin_once(d, timeout_sec=0.5)
        if d.get_clock().now().nanoseconds - t0 > 10e9:
            print("no /scan in 10 s - is the launch running?")
            return
    check_scan(d.scan)
    check_straightness(d.scan)

    print(f"\n--- 3/4. now DRIVE A FULL LAP ({args.seconds:.0f} s) ---")
    end = d.get_clock().now().nanoseconds + args.seconds * 1e9
    while d.get_clock().now().nanoseconds < end:
        rclpy.spin_once(d, timeout_sec=0.02)
        d.sample()

    print("\n--- 3. where does the heading error live? ---")
    print(f"  odom  yaw turned, total : {math.degrees(d.odom_acc):+9.1f} deg  (raw odometry)")
    print(f"  map   yaw turned, total : {math.degrees(d.map_acc):+9.1f} deg  (after SLAM)")
    print(f"  difference              : {math.degrees(d.map_acc - d.odom_acc):+9.1f} deg"
          "   <- how much SLAM corrected")
    print("  If you drove exactly one closed lap, each should read about +-360.")
    print("  Both near 380 (or 340) => scan matching inherited the error, not odometry.")
    print("  Only odom off, map near 360 => odometry drifts and SLAM is fixing it.")

    print("\n--- 4. did loop closure ever fire? ---")
    if d.jumps:
        big = [j for j in d.jumps if j > 5.0]
        print(f"  {len(d.jumps)} correction step(s) > 0.75 deg in map->odom")
        print(f"  largest: {max(d.jumps):.2f} deg")
        if big:
            print(f"  {len(big)} of them exceed 5 deg => genuine loop closure(s) fired.")
        else:
            print("  ALL are small. These are ordinary incremental scan-match")
            print("  corrections, NOT a loop closure. A closure that erased a lap's")
            print("  worth of accumulated drift would step by roughly that whole")
            print("  drift at once. The lap is not closing.")
    else:
        print("  NO step changes in map->odom over the whole run.")
        print("  => the lap never closed. Drift is never undone, so every lap")
        print("     lands as its own rotated copy of the track.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
