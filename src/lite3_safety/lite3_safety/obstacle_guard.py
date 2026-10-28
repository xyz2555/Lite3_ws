#!/usr/bin/env python3
"""obstacle_guard: stops the Lite3 when something is too close.

    teleop / your node --> cmd_vel --> [obstacle_guard] --> /lite3/cmd_vel_safe --> lite3_driver

Distance sources (enable one or both):
  * built-in ultrasound   /lite3/ultrasound/front|back  (sensor_msgs/Range, from lite3_driver)
  * depth camera          e.g. RealSense /camera/camera/depth/image_rect_raw (sensor_msgs/Image)

Forward motion uses the nearest *front* distance from all fresh sources, backward
motion uses the back ultrasound. Inside ``slow_distance`` speed is scaled down,
inside ``stop_distance`` it is set to zero. Turning and side-stepping stay allowed
so the operator can steer away (configurable).

Publishes
  /lite3/obstacle/front_distance  std_msgs/Float32   (-1 = no data, inf = clear)
  /lite3/obstacle/blocked         std_msgs/Bool      (forward motion blocked)
  /lite3/obstacle/blocked_back    std_msgs/Bool      (backward motion blocked)
Services
  ~/enable                        std_srvs/SetBool   (bypass the guard when false)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

from lite3_safety.guard_logic import (GateConfig, SpeedGate, depth_to_meters,
                                      gate_command, nearest_in_roi)


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__("obstacle_guard")
        dp = self.declare_parameter
        dp("enabled", True)
        dp("cmd_vel_in", "cmd_vel")
        dp("cmd_vel_out", "/lite3/cmd_vel_safe")
        dp("stop_distance", 0.5)
        dp("slow_distance", 1.0)
        dp("hysteresis", 0.1)
        dp("allow_lateral_when_blocked", True)
        dp("allow_rotation_when_blocked", True)
        dp("sensor_timeout", 0.5)
        dp("fail_safe", True)
        dp("cmd_timeout", 0.5)
        dp("rate_hz", 20.0)
        # ultrasound source
        dp("use_ultrasound", True)
        dp("ultrasound_front_topic", "/lite3/ultrasound/front")
        dp("ultrasound_back_topic", "/lite3/ultrasound/back")
        # depth source
        dp("use_depth", False)
        dp("depth_topic", "/camera/camera/depth/image_rect_raw")
        dp("depth_roi", [0.3, 0.7, 0.2, 0.65])   # x_min, x_max, y_min, y_max (fractions)
        dp("depth_min_valid", 0.15)
        dp("depth_max_valid", 4.0)
        dp("depth_percentile", 5.0)
        dp("depth_min_pixels", 50)
        dp("depth_offset", 0.0)   # camera-to-front-of-robot distance to subtract (m)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.enabled = bool(g("enabled"))
        self.cfg = GateConfig(
            stop_distance=float(g("stop_distance")),
            slow_distance=float(g("slow_distance")),
            hysteresis=float(g("hysteresis")),
            allow_lateral=bool(g("allow_lateral_when_blocked")),
            allow_rotation=bool(g("allow_rotation_when_blocked")),
        )
        self.front_gate = SpeedGate(self.cfg)
        self.back_gate = SpeedGate(self.cfg)
        self.front_scale = 1.0
        self.back_scale = 1.0
        self.sensor_timeout = float(g("sensor_timeout"))
        self.fail_safe = bool(g("fail_safe"))
        self.cmd_timeout = float(g("cmd_timeout"))
        self.depth_roi = tuple(float(v) for v in g("depth_roi"))
        self.depth_args = dict(
            min_valid=float(g("depth_min_valid")),
            max_valid=float(g("depth_max_valid")),
            percentile=float(g("depth_percentile")),
            min_pixels=int(g("depth_min_pixels")),
        )
        self.depth_offset = float(g("depth_offset"))

        # latest readings: name -> (distance or None, time)
        self.front_src = {}
        self.back_src = {}
        self.use_us = bool(g("use_ultrasound"))
        self.use_depth = bool(g("use_depth"))
        if not (self.use_us or self.use_depth):
            self.get_logger().warn("no distance source enabled, guard only passes commands through")

        if self.use_us:
            self.front_src["ultrasound"] = (None, None)
            self.back_src["ultrasound"] = (None, None)
            self.create_subscription(Range, g("ultrasound_front_topic"),
                                     lambda m: self._on_range(m, self.front_src),
                                     qos_profile_sensor_data)
            self.create_subscription(Range, g("ultrasound_back_topic"),
                                     lambda m: self._on_range(m, self.back_src),
                                     qos_profile_sensor_data)
        if self.use_depth:
            self.front_src["depth"] = (None, None)
            self.create_subscription(Image, g("depth_topic"), self._on_depth,
                                     qos_profile_sensor_data)

        self.last_cmd = None
        self.last_cmd_time = None
        self._last_blocked = (False, False)
        self._back_dist = None

        self.out_pub = self.create_publisher(Twist, g("cmd_vel_out"), 10)
        self.dist_pub = self.create_publisher(Float32, "/lite3/obstacle/front_distance", 10)
        self.blocked_pub = self.create_publisher(Bool, "/lite3/obstacle/blocked", 10)
        self.blocked_back_pub = self.create_publisher(Bool, "/lite3/obstacle/blocked_back", 10)
        self.create_subscription(Twist, g("cmd_vel_in"), self._on_cmd, 10)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)

        self.get_logger().info(
            f"obstacle_guard: '{g('cmd_vel_in')}' -> '{g('cmd_vel_out')}', "
            f"stop {self.cfg.stop_distance} m, slow {self.cfg.slow_distance} m, "
            f"sources: {', '.join(self.front_src) or 'none'}")

    # ------------------------------------------------------------ sensors
    def _now(self):
        return self.get_clock().now()

    def _on_range(self, msg: Range, store: dict):
        r = msg.range
        if r == -math.inf:           # REP 117: object closer than min_range
            d = 0.0
        elif r == math.inf:          # REP 117: nothing within max_range
            d = math.inf
        elif not math.isfinite(r) or r < msg.min_range:
            d = None
        elif r > msg.max_range:
            d = math.inf
        else:
            d = float(r)
        store["ultrasound"] = (d, self._now())

    def _on_depth(self, msg: Image):
        try:
            depth = depth_to_meters(msg.encoding, msg.height, msg.width, msg.step,
                                    bytes(msg.data), bool(msg.is_bigendian))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        d = nearest_in_roi(depth, self.depth_roi, **self.depth_args)
        if d is not None and math.isfinite(d):
            d = max(0.0, d - self.depth_offset)
        self.front_src["depth"] = (d, self._now())

    def _nearest(self, store: dict):
        """Nearest distance among fresh sources; None if no source is fresh."""
        if not store:
            return math.inf          # no source configured for this direction
        now, best, fresh = self._now(), math.inf, False
        for d, t in store.values():
            if t is None or (now - t).nanoseconds * 1e-9 > self.sensor_timeout:
                continue
            if d is None:
                continue
            fresh = True
            best = min(best, d)
        return best if fresh else None

    # ------------------------------------------------------------ gating
    def _gated(self, cmd: Twist) -> Twist:
        if not self.enabled:
            return cmd
        vx, vy, wz = gate_command(
            cmd.linear.x, cmd.linear.y, cmd.angular.z,
            self.front_scale, self.back_scale,
            self.front_gate.blocked, self.back_gate.blocked, self.cfg)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        return out

    def _update_gates(self):
        front = self._nearest(self.front_src)
        back = self._nearest(self.back_src)
        self.front_scale = self.front_gate.update(front, self.fail_safe)
        self.back_scale = self.back_gate.update(back, self.fail_safe)
        self._back_dist = back
        return front

    def _on_cmd(self, msg: Twist):
        self.last_cmd, self.last_cmd_time = msg, self._now()
        self._update_gates()
        self.out_pub.publish(self._gated(msg))

    def _tick(self):
        front = self._update_gates()
        blocked = (self.front_gate.blocked, self.back_gate.blocked)

        self.dist_pub.publish(Float32(data=-1.0 if front is None else float(front)))
        self.blocked_pub.publish(Bool(data=self.enabled and blocked[0]))
        self.blocked_back_pub.publish(Bool(data=self.enabled and blocked[1]))

        if blocked != self._last_blocked:
            # if any(blocked) and self.enabled:
            #     why = "no fresh sensor data" if front is None else f"{front:.2f} m"
            #     side = "front" if blocked[0] else "back"
            #     self.get_logger().warn(f"BLOCKED ({side}): {why}")
            # else:
            #     self.get_logger().info("path clear")
             # log each side with ITS OWN distance (the old message always printed the front one)
            for name, now_b, was_b, dist in (("front", blocked[0], self._last_blocked[0], front),
                                             ("back", blocked[1], self._last_blocked[1],
                                              self._back_dist)):
                if now_b and not was_b and self.enabled:
                    why = "no fresh sensor data" if dist is None else f"{dist:.2f} m"
                    self.get_logger().warn(f"BLOCKED ({name}): {why}")
                elif was_b and not now_b:
                    self.get_logger().info(f"{name} clear")
            # react immediately instead of waiting for the next cmd_vel
            if self.last_cmd is not None and \
                    (self._now() - self.last_cmd_time).nanoseconds * 1e-9 < self.cmd_timeout:
                self.out_pub.publish(self._gated(self.last_cmd))
        self._last_blocked = blocked

    def _on_enable(self, req, resp):
        self.enabled = bool(req.data)
        resp.success = True
        resp.message = "guard enabled" if self.enabled else "guard BYPASSED"
        self.get_logger().warn(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.out_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
