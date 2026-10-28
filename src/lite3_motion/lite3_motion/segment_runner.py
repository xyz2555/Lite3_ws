"""SegmentRunner: plays timed velocity segments on cmd_vel without blocking.

Everything runs from one timer, so the executor stays free (no time.sleep in
callbacks). If /lite3/obstacle/blocked (front) or .../blocked_back is true
while a segment moves in that direction, the runner publishes zero and freezes
that segment's clock until the path is clear again, so an open-loop plan does
not "use up" its time while waiting.
"""

from typing import List, Optional

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from lite3_motion.plan import Segment


class SegmentRunner(Node):
    def __init__(self, name: str, segments: Optional[List[Segment]] = None):
        super().__init__(name)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("blocked_topic", "/lite3/obstacle/blocked")
        self.declare_parameter("blocked_back_topic", "/lite3/obstacle/blocked_back")
        self.declare_parameter("start_delay", 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.pub = self.create_publisher(Twist, g("cmd_vel_topic"), 10)
        self.create_subscription(Bool, g("blocked_topic"), self._on_blocked, 10)
        self.create_subscription(Bool, g("blocked_back_topic"), self._on_blocked_back, 10)
        self.dt = 1.0 / float(g("publish_rate_hz"))
        self.start_ticks = int(float(g("start_delay")) / self.dt)

        self.segments: List[Segment] = list(segments or [])
        self.idx = 0
        self.elapsed = 0.0
        self.blocked = False
        self.blocked_back = False
        self.done = False
        self._was_paused = False
        self._timer = None

    def start(self, segments: Optional[List[Segment]] = None):
        if segments is not None:
            self.segments = list(segments)
        total = sum(s.duration for s in self.segments)
        self.get_logger().info(f"plan: {len(self.segments)} segments, ~{total:.1f} s")
        self._timer = self.create_timer(self.dt, self._tick)

    def _on_blocked(self, msg: Bool):
        self.blocked = bool(msg.data)

    def _on_blocked_back(self, msg: Bool):
        self.blocked_back = bool(msg.data)

    def _is_blocked(self, seg: Segment) -> bool:
        return (seg.vx > 0 and self.blocked) or (seg.vx < 0 and self.blocked_back)

    def stop(self):
        self.pub.publish(Twist())

    def _tick(self):
        if self.start_ticks > 0:            # let publishers/subscribers match
            self.start_ticks -= 1
            return
        if self.idx >= len(self.segments):
            if not self.done:
                self.stop()
                self.done = True
                self.get_logger().info("plan complete")
            return

        seg = self.segments[self.idx]
        if self.elapsed == 0.0 and not self._was_paused:
            self.get_logger().info(f"[{self.idx + 1}/{len(self.segments)}] {seg.name} "
                                   f"({seg.duration:.2f} s)")

        if self._is_blocked(seg):
            if not self._was_paused:
                self.get_logger().warn("obstacle in the way, pausing plan")
            self._was_paused = True
            self.stop()
            return
        if self._was_paused:
            self.get_logger().info("path clear, resuming")
            self._was_paused = False

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = seg.vx, seg.vy, seg.wz
        self.pub.publish(msg)
        self.elapsed += self.dt
        if self.elapsed >= seg.duration:
            self.idx += 1
            self.elapsed = 0.0
