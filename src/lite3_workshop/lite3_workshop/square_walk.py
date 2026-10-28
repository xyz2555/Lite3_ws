"""Lab 6: open-loop square on the real Lite3.

  ros2 run lite3_workshop square_walk --ros-args -p topic:=/cmd_vel

Publishes to /cmd_vel by default, which goes through obstacle_guard before
reaching the robot when the bringup runs with use_guard:=true.
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node

from lite3_workshop.common import run

# (vx m/s, wz rad/s, seconds): 0.8 m forward, ~90 deg left, four times
PLAN = [(0.2, 0.0, 4.0), (0.0, 0.4, 3.9)] * 4


class SquareWalk(Node):
    def __init__(self):
        super().__init__("square_walk")
        self.declare_parameter("topic", "/cmd_vel")
        topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(Twist, topic, 10)
        self.step, self.t0 = 0, self.now()
        self.create_timer(0.1, self.tick)  # 10 Hz

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        if self.step >= len(PLAN):
            self.pub.publish(Twist())
            return
        vx, wz, dur = PLAN[self.step]
        if self.now() - self.t0 > dur:
            self.step, self.t0 = self.step + 1, self.now()
            self.get_logger().info(f"step {self.step}")
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self.pub.publish(msg)


def main(args=None):
    run(SquareWalk, args)


if __name__ == "__main__":
    main()
