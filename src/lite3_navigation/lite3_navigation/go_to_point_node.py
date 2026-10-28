#!/usr/bin/env python3
"""go_to_point: drive the Lite3 to a pose clicked with RViz2's "2D Goal Pose".

    RViz2 2D Goal Pose --> /goal_pose --> [go_to_point] --> cmd_vel_topic --> (mux) --> obstacle_guard --> driver
                                              ^   ^   ^
                       TF odom->base_link ----+   |   +---- /lite3/basic_state (standing?)
                       /lite3/obstacle/blocked ---+

The pose comes from TF odom->base_link published by lite3_driver (position from the
robot's leg odometry, yaw from its IMU/estimator), so the IMU is already inside it.
The goal is converted into the odom frame, so RViz's Fixed Frame can be odom (or any
frame connected to it by TF).

Publishes
  cmd_vel_topic            geometry_msgs/Twist   only while a goal is active (+ one zero at the end)
  /lite3/goto/status       std_msgs/String       IDLE | ROTATE | MOVE | WAIT_BLOCKED | ALIGN |
                                                 SUCCEEDED | ABORTED:<reason>
Services
  ~/cancel                 std_srvs/Trigger      cancel the current goal
  ~/enable                 std_srvs/SetBool      false = cancel and ignore new goals

Why it does not publish while idle: when teleop and this node share /cmd_vel, a zero
command at 20 Hz would fight with the operator. With the optional twist_mux the same
holds for the mux timeouts.
"""

import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from lite3_navigation.goto_logic import (GoToConfig, GoToPoint, State, wrap_angle,
                                         yaw_from_quat)


class GoToPointNode(Node):
    def __init__(self):
        super().__init__("go_to_point")
        dp = self.declare_parameter
        dp("odom_frame", "odom")
        dp("base_frame", "base_link")
        dp("goal_topic", "/goal_pose")
        dp("cmd_topic", "/cmd_vel")
        dp("teleop_topic", "")                      # e.g. /cmd_vel_teleop: operator input cancels the goal
        dp("blocked_topic", "/lite3/obstacle/blocked")
        dp("state_topic", "/lite3/basic_state")
        dp("status_topic", "/lite3/goto/status")
        dp("rate_hz", 20.0)
        dp("pose_timeout", 0.3)                     # s, older TF means "pose_stale"
        dp("require_standing", True)
        dp("standing_states", [6])                  # basic_state values meaning "standing"
        dp("teleop_deadband", 1e-3)
        for f in fields(GoToConfig):                # every tuning value is a parameter
            dp(f.name, f.default)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odom_frame, self.base_frame = gp("odom_frame"), gp("base_frame")
        self.pose_timeout = float(gp("pose_timeout"))
        self.require_standing = bool(gp("require_standing"))
        self.standing = set(int(v) for v in gp("standing_states"))
        self.teleop_deadband = float(gp("teleop_deadband"))
        cfg = GoToConfig(**{f.name: float(gp(f.name)) for f in fields(GoToConfig)})
        self.ctrl = GoToPoint(cfg)

        self.enabled = True
        self.blocked = False
        self.basic_state = None
        self._was_active = False
        self._last_status = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, gp("cmd_topic"), 10)
        self.status_pub = self.create_publisher(String, gp("status_topic"), 10)
        self.create_subscription(PoseStamped, gp("goal_topic"), self.on_goal, 10)
        self.create_subscription(Bool, gp("blocked_topic"), self.on_blocked, 10)
        self.create_subscription(Int32, gp("state_topic"), self.on_state, 10)
        if gp("teleop_topic"):
            self.create_subscription(Twist, gp("teleop_topic"), self.on_teleop, 10)
        self.create_service(Trigger, "~/cancel", self.srv_cancel)
        self.create_service(SetBool, "~/enable", self.srv_enable)

        self.create_timer(1.0 / float(gp("rate_hz")), self.control)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            f"go_to_point up: goals on '{gp('goal_topic')}', commands to '{gp('cmd_topic')}', "
            f"pose from TF {self.odom_frame}->{self.base_frame}, "
            f"require_standing={self.require_standing} {sorted(self.standing)}")

    # ---------------------------------------------------------- helpers
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def current_pose(self):
        """(x, y, yaw) in the odom frame, or None if TF is missing, stale or not finite."""
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception:                                    # noqa: BLE001 (any TF error = no pose)
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.pose_timeout:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        pose = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        return pose if all(math.isfinite(v) for v in pose) else None

    def goal_in_odom(self, msg: PoseStamped):
        """Return (x, y, yaw or None) of the goal expressed in the odom frame, or None."""
        frame = msg.header.frame_id or self.odom_frame
        p, q = msg.pose.position, msg.pose.orientation
        has_yaw = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-6
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w) if has_yaw else None
        x, y = p.x, p.y
        if frame != self.odom_frame:
            try:
                # no timeout: blocking inside a callback would stop TF from being received
                tf = self.tf_buffer.lookup_transform(self.odom_frame, frame, Time())
            except Exception as e:                           # noqa: BLE001
                self.get_logger().error(f"cannot express goal frame '{frame}' in "
                                        f"'{self.odom_frame}': {e}")
                return None
            tq, tt = tf.transform.rotation, tf.transform.translation
            tyaw = yaw_from_quat(tq.x, tq.y, tq.z, tq.w)     # 2D: roll/pitch ignored
            c, s = math.cos(tyaw), math.sin(tyaw)
            x, y = tt.x + c * p.x - s * p.y, tt.y + s * p.x + c * p.y
            yaw = None if yaw is None else wrap_angle(yaw + tyaw)
        if not all(math.isfinite(v) for v in (x, y)) or (yaw is not None and not math.isfinite(yaw)):
            return None
        return x, y, yaw

    def stop_robot(self):
        self.pub.publish(Twist())

    # ---------------------------------------------------------- subscriptions
    def on_blocked(self, msg: Bool):
        self.blocked = bool(msg.data)

    def on_state(self, msg: Int32):
        self.basic_state = int(msg.data)

    def on_teleop(self, msg: Twist):
        moving = max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)) > self.teleop_deadband
        if moving and self.ctrl.active:
            self.get_logger().warn("operator input detected: goal cancelled (teleop_override)")
            self.ctrl.cancel("teleop_override")

    def on_goal(self, msg: PoseStamped):
        if not self.enabled:
            self.get_logger().warn("goal ignored: go_to_point is disabled")
            return
        pose = self.current_pose()
        if pose is None:
            self.get_logger().error("goal rejected: no fresh pose from TF "
                                    f"{self.odom_frame}->{self.base_frame} (is the driver running?)")
            return
        if self.require_standing and self.basic_state not in self.standing:
            hint = ("no /lite3/basic_state received yet (is the patched driver running?)"
                    if self.basic_state is None else f"robot state is {self.basic_state}")
            self.get_logger().error(
                f"goal rejected: {hint}; expected one of {sorted(self.standing)} "
                "(stand up first, or set require_standing:=false)")
            return
        goal = self.goal_in_odom(msg)
        if goal is None:
            self.get_logger().error("goal rejected: invalid goal pose")
            return
        gx, gy, gyaw = goal
        self.ctrl.set_goal(gx, gy, gyaw, pose, self.now_s())
        yaw_txt = "none" if gyaw is None else f"{math.degrees(gyaw):.0f} deg"
        self.get_logger().info(
            f"new goal in {self.odom_frame}: x={gx:.2f} y={gy:.2f} yaw={yaw_txt} "
            f"(from x={pose[0]:.2f} y={pose[1]:.2f} yaw={math.degrees(pose[2]):.0f} deg)")

    # ---------------------------------------------------------- services
    def srv_cancel(self, _req, resp):
        was = self.ctrl.active
        self.ctrl.cancel("cancelled")
        resp.success, resp.message = True, "goal cancelled" if was else "no active goal"
        return resp

    def srv_enable(self, req, resp):
        self.enabled = bool(req.data)
        if not self.enabled:
            self.ctrl.cancel("disabled")
        resp.success, resp.message = True, "enabled" if self.enabled else "disabled"
        return resp

    # ---------------------------------------------------------- control loop
    def control(self):
        pose = self.current_pose()
        fault = None
        if self.ctrl.active:
            if pose is None:
                fault = "pose_stale"
            elif self.require_standing and self.basic_state not in self.standing:
                fault = "not_standing"
        cmd = self.ctrl.step(self.now_s(), pose, blocked=self.blocked, fault=fault)

        if self.ctrl.active or self._was_active:             # publish only around a goal
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = cmd.vx, cmd.vy, cmd.wz
            self.pub.publish(msg)
        if self._was_active and not self.ctrl.active:
            self.get_logger().info(f"goal finished: {self.ctrl.status()}")
            self.publish_status()
        self._was_active = self.ctrl.active

    def publish_status(self):
        text = self.ctrl.status()
        if self.ctrl.active:
            text += (f" dist={self.ctrl.dist:.2f} m heading_err="
                     f"{math.degrees(self.ctrl.heading_err):.0f} deg")
        if self.ctrl.state in (State.ALIGN, State.SUCCEEDED):
            text += f" yaw_err={math.degrees(self.ctrl.yaw_err):.0f} deg"
        self.status_pub.publish(String(data=text))


def main(args=None):
    # NO signal handler: Ctrl+C stays a KeyboardInterrupt so the final zero Twist can be sent
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GoToPointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ctrl.active:
                node.stop_robot()
        except Exception:                                    # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
