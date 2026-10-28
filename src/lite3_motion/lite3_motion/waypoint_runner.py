#!/usr/bin/env python3
"""JSON waypoint runner.

  ros2 run lite3_motion waypoint_runner --ros-args -p waypoints_file:=/path/plan.json

Each waypoint is a RELATIVE body-frame motion (rotate ``yaw`` deg first, then
move ``x``/``y`` m). Open-loop, timed. Pauses while an obstacle blocks the path.
"""

import rclpy
from rclpy.signals import SignalHandlerOptions

from lite3_motion.plan import plan_from_file
from lite3_motion.segment_runner import SegmentRunner


class WaypointRunner(SegmentRunner):
    def __init__(self):
        super().__init__("lite3_waypoint_runner")
        self.declare_parameter("waypoints_file", "")
        path = self.get_parameter("waypoints_file").value
        if not path:
            raise RuntimeError("parameter 'waypoints_file' is empty")
        self.get_logger().info(f"loading {path}")
        self.start(plan_from_file(path))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = WaypointRunner()
    except (RuntimeError, ValueError, OSError) as e:
        rclpy.logging.get_logger("lite3_waypoint_runner").error(str(e))
        rclpy.shutdown()
        return
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
