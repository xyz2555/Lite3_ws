#!/usr/bin/env python3
"""One-shot relative move.

  ros2 run lite3_motion move_cli --x 1.0               # forward 1 m
  ros2 run lite3_motion move_cli --yaw 90              # turn left 90 deg
  ros2 run lite3_motion move_cli --x 0.5 --y 0.2 --yaw -45 --speed 0.2

Rotation first, then translation. Exits when done. Pauses while
/lite3/obstacle/blocked is true.
"""

import argparse
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from lite3_motion.plan import motion_segments
from lite3_motion.segment_runner import SegmentRunner


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    ap = argparse.ArgumentParser(prog="move_cli")
    ap.add_argument("--x", type=float, default=0.0, help="forward (+) / backward (-) m")
    ap.add_argument("--y", type=float, default=0.0, help="left (+) / right (-) m")
    ap.add_argument("--yaw", type=float, default=0.0, help="degrees, CCW (left) positive")
    ap.add_argument("--speed", type=float, default=0.3, help="linear m/s (max 1.0)")
    ap.add_argument("--wspeed", type=float, default=0.5, help="angular rad/s (max 1.5)")
    a = ap.parse_args(argv)

    segs = motion_segments("move", a.x, a.y, a.yaw, a.speed, a.wspeed)
    if not segs:
        print("nothing to do (all deltas are zero)")
        return

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SegmentRunner("lite3_move_cli")
    node.start(segs)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
