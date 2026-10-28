#!/usr/bin/env python3
"""fake_lite3: a stand-in for lite3_driver + robot, to try go_to_point without hardware.

Subscribes   cmd_vel (geometry_msgs/Twist)           default /cmd_vel
Publishes    TF odom -> base_link                    (same as the real driver)
             /lite3/odom (nav_msgs/Odometry)
             /lite3/basic_state (std_msgs/Int32)     constant 6 = "standing"
             /lite3/obstacle/blocked (std_msgs/Bool) like obstacle_guard (virtual obstacle)

The body follows the commanded velocity with a first-order lag. An optional virtual
obstacle (obstacle_x / obstacle_y in the odom frame) sets "blocked" when it is within
``stop_distance`` in front of the robot and also zeroes forward speed, like the guard.

NEVER run it together with the real lite3_driver: both would publish odom->base_link.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Int32
from tf2_ros import TransformBroadcaster


class FakeLite3(Node):
    def __init__(self):
        super().__init__("fake_lite3")
        dp = self.declare_parameter
        dp("cmd_topic", "/cmd_vel")
        dp("odom_frame", "odom")
        dp("base_frame", "base_link")
        dp("rate_hz", 50.0)
        dp("tau", 0.3)                       # s, velocity lag
        dp("cmd_timeout", 0.5)               # s, like the real driver
        dp("x0", 0.0)
        dp("y0", 0.0)
        dp("yaw0", 0.0)
        dp("basic_state", 6)
        dp("obstacle_x", float("nan"))       # NaN = no virtual obstacle
        dp("obstacle_y", 0.0)
        dp("stop_distance", 0.5)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odom_frame, self.base_frame = gp("odom_frame"), gp("base_frame")
        self.tau, self.cmd_timeout = float(gp("tau")), float(gp("cmd_timeout"))
        self.state_value = int(gp("basic_state"))
        self.ox, self.oy = float(gp("obstacle_x")), float(gp("obstacle_y"))
        self.stop_distance = float(gp("stop_distance"))
        self.x, self.y, self.yaw = float(gp("x0")), float(gp("y0")), float(gp("yaw0"))
        self.vx = self.vy = self.wz = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.dt = 1.0 / float(gp("rate_hz"))

        self.create_subscription(Twist, gp("cmd_topic"), self.on_cmd, 10)
        self.tf_br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/lite3/odom", 10)
        self.state_pub = self.create_publisher(Int32, "/lite3/basic_state", 10)
        self.blocked_pub = self.create_publisher(Bool, "/lite3/obstacle/blocked", 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"fake_lite3 up at x={self.x:.2f} y={self.y:.2f}, cmd from '{gp('cmd_topic')}'")

    def on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.cmd_time = self.get_clock().now()

    def obstacle_ahead(self) -> bool:
        if math.isnan(self.ox):
            return False
        dx, dy = self.ox - self.x, self.oy - self.y
        ahead = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        side = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        return 0.0 < ahead < self.stop_distance and abs(side) < 0.4

    def tick(self):
        now = self.get_clock().now()
        vx, vy, wz = self.cmd
        if self.cmd_time is None or (now - self.cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            vx = vy = wz = 0.0
        blocked = self.obstacle_ahead()
        if blocked and vx > 0.0:
            vx = 0.0
        a = self.dt / (self.tau + self.dt)
        self.vx += a * (vx - self.vx)
        self.vy += a * (vy - self.vy)
        self.wz += a * (wz - self.wz)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx * c - self.vy * s) * self.dt
        self.y += (self.vx * s + self.vy * c) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * self.dt),
                              math.cos(self.yaw + self.wz * self.dt))

        stamp = now.to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        t = TransformStamped()
        t.header.stamp, t.header.frame_id = stamp, self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_br.sendTransform(t)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.vx, self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        self.state_pub.publish(Int32(data=self.state_value))
        self.blocked_pub.publish(Bool(data=blocked))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FakeLite3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
