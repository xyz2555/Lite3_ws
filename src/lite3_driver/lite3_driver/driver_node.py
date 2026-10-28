#!/usr/bin/env python3
"""lite3_driver: UDP bridge between ROS 2 and the Lite3 motion host.

Subscribes
  cmd_vel (geometry_msgs/Twist)        topic name from param ``cmd_vel_topic``

Publishes
  /lite3/odom              nav_msgs/Odometry   pose from pos_world, twist from vel_body
  /lite3/imu               sensor_msgs/Imu     from rpy / rpy_vel / xyz_acc
  /lite3/ultrasound/front  sensor_msgs/Range   built-in ultrasound (if the unit reports it)
  /lite3/ultrasound/back   sensor_msgs/Range
  /lite3/basic_state       std_msgs/Int32      robot basic_state (seen: 1 = sitting, 6 = standing)
  TF odom -> base_link     (param ``publish_tf``)

Services (std_srvs/Trigger)
  /lite3/stand_up, /lite3/sit_down, /lite3/enter_nav, /lite3/exit_nav, /lite3/estop

Safety
  * heartbeat at ``heartbeat_hz`` (protocol requires >= 2 Hz)
  * last velocity is resent at ``resend_hz`` while fresh, and a zero command
    is sent once when no cmd_vel arrives for ``cmd_timeout`` seconds
"""

import math
import socket
import threading

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from lite3_driver import protocol as p


class Lite3Driver(Node):
    def __init__(self):
        super().__init__("lite3_driver")

        dp = self.declare_parameter
        dp("robot_ip", "192.168.1.120")
        dp("robot_port", 43893)
        dp("local_port", 43897)          # must match the port in network.toml
        dp("cmd_vel_topic", "cmd_vel")
        dp("cmd_timeout", 0.5)
        dp("resend_hz", 20.0)
        dp("heartbeat_hz", 4.0)
        dp("publish_rate_hz", 50.0)
        dp("odom_frame", "odom")
        dp("base_frame", "base_link")
        dp("publish_tf", True)
        dp("invert_wz", True)  
        dp("rpy_in_degrees", True)      # verify on your firmware: tilt the robot ~10 deg
        dp("ultrasound_offset", -1)      # -1 = read the last 16 bytes of 0x0901
        dp("ultrasound_max_range", 4.0)
        dp("ultrasound_fov", 0.5)        # rad, informational for RViz
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.robot_addr = (gp("robot_ip"), int(gp("robot_port")))
        self.cmd_timeout = float(gp("cmd_timeout"))
        self.odom_frame = gp("odom_frame")
        self.base_frame = gp("base_frame")
        self.publish_tf = bool(gp("publish_tf"))
        self.rpy_deg = bool(gp("rpy_in_degrees"))
        self.invert_wz = bool(gp("invert_wz"))
        self.us_offset = int(gp("ultrasound_offset"))
        self.us_max = float(gp("ultrasound_max_range"))
        self.us_fov = float(gp("ultrasound_fov"))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", int(gp("local_port"))))
        self.sock.settimeout(0.2)

        # shared with the rx thread
        self._lock = threading.Lock()
        self._state = None
        self._state_fresh = False

        # last command
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = None
        self._stopped = True

        self.create_subscription(Twist, gp("cmd_vel_topic"), self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/lite3/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/lite3/imu", 10)
        self.us_front_pub = self.create_publisher(Range, "/lite3/ultrasound/front", 10)
        self.us_back_pub = self.create_publisher(Range, "/lite3/ultrasound/back", 10)
        self.basic_state_pub = self.create_publisher(Int32, "/lite3/basic_state", 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        self.create_service(Trigger, "/lite3/stand_up", self.srv_stand)
        self.create_service(Trigger, "/lite3/sit_down", self.srv_sit)
        self.create_service(Trigger, "/lite3/enter_nav", self.srv_enter_nav)
        self.create_service(Trigger, "/lite3/exit_nav", self.srv_exit_nav)
        self.create_service(Trigger, "/lite3/estop", self.srv_estop)

        self.create_timer(1.0 / float(gp("heartbeat_hz")), self.send_heartbeat)
        self.create_timer(1.0 / float(gp("resend_hz")), self.velocity_tick)
        self.create_timer(1.0 / float(gp("publish_rate_hz")), self.publish_state)
        self._nav_timer = None

        self._running = True
        self._rx = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx.start()

        self.get_logger().info(
            f"lite3_driver up: robot {self.robot_addr[0]}:{self.robot_addr[1]}, "
            f"listening on :{gp('local_port')}, cmd_vel <- '{gp('cmd_vel_topic')}'")

    # ---------------------------------------------------------- tx helpers
    def _send(self, data: bytes, label: str):
        try:
            self.sock.sendto(data, self.robot_addr)
        except OSError as e:
            self.get_logger().warn(f"send {label} failed: {e}", throttle_duration_sec=2.0)

    def _send_velocity(self, vx, vy, wz):
        for pkt in p.pack_velocity(vx, vy, wz, invert_wz=self.invert_wz):
            self._send(pkt, "velocity")

    def send_heartbeat(self):
        self._send(p.pack_simple(p.CMD_HEARTBEAT), "heartbeat")

    def stop_robot(self):
        self._cmd, self._cmd_time = (0.0, 0.0, 0.0), None
        self._send_velocity(0.0, 0.0, 0.0)
        self._stopped = True

    # ---------------------------------------------------------- cmd_vel
    def on_cmd_vel(self, msg: Twist):
        self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_time = self.get_clock().now()
        self._stopped = False
        self._send_velocity(*self._cmd)

    def velocity_tick(self):
        if self._cmd_time is None:
            return
        age = (self.get_clock().now() - self._cmd_time).nanoseconds * 1e-9
        if age <= self.cmd_timeout:
            self._send_velocity(*self._cmd)
        elif not self._stopped:
            self.get_logger().info("cmd_vel timeout, stopping robot")
            self.stop_robot()

    # ---------------------------------------------------------- services
    def _ok(self, resp, msg):
        resp.success, resp.message = True, msg
        self.get_logger().info(msg)
        return resp

    def srv_stand(self, _req, resp):
        self._send(p.pack_simple(p.CMD_STAND_SIT), "stand/sit")
        return self._ok(resp, "stand/sit toggle sent (stands if sitting)")

    def srv_sit(self, _req, resp):
        self.stop_robot()
        self._send(p.pack_simple(p.CMD_STAND_SIT), "stand/sit")
        return self._ok(resp, "stand/sit toggle sent (sits if standing)")

    def srv_enter_nav(self, _req, resp):
        # move mode first, navigation 100 ms later, without blocking the executor
        self._send(p.pack_simple(p.CMD_MOVE_MODE), "move mode")
        if self._nav_timer is not None:
            self._nav_timer.cancel()
        self._nav_timer = self.create_timer(0.1, self._send_nav_once)
        return self._ok(resp, "move + navigation mode requested, cmd_vel active")

    def _send_nav_once(self):
        self._send(p.pack_simple(p.CMD_NAVIGATION), "navigation")
        self._nav_timer.cancel()
        self.destroy_timer(self._nav_timer)
        self._nav_timer = None

    def srv_exit_nav(self, _req, resp):
        self.stop_robot()
        self._send(p.pack_simple(p.CMD_MANUAL), "manual")
        return self._ok(resp, "manual mode, cmd_vel ignored by robot")

    def srv_estop(self, _req, resp):
        self.stop_robot()
        self._send(p.pack_simple(p.CMD_ESTOP), "estop")
        self.get_logger().warn("SOFT E-STOP SENT")
        resp.success, resp.message = True, "soft emergency stop sent"
        return resp

    # ---------------------------------------------------------- rx
    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    continue
                return
            parts = p.split_packet(data)
            if parts is None:
                continue
            code, ctype, body = parts
            if code == p.RECV_ROBOT_STATE and ctype == 1:
                st = p.parse_robot_state(body, self.us_offset, self.us_max)
                if st is not None:
                    with self._lock:
                        self._state, self._state_fresh = st, True

    # ---------------------------------------------------------- publish
    def publish_state(self):
        with self._lock:
            if not self._state_fresh:
                return
            st, self._state_fresh = self._state, False

        stamp = self.get_clock().now().to_msg()
        k = math.pi / 180.0 if self.rpy_deg else 1.0
        roll, pitch, yaw_imu = (v * k for v in st.rpy)
        q_imu = p.quaternion_from_rpy(roll, pitch, yaw_imu)

        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, self.base_frame
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q_imu
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
            v for v in st.rpy_vel)
        (imu.linear_acceleration.x, imu.linear_acceleration.y,
         imu.linear_acceleration.z) = st.xyz_acc
        self.imu_pub.publish(imu)

        x, y, _z_height = st.pos_world
        q = p.quaternion_from_rpy(0.0, 0.0, yaw_imu)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = st.vel_body[0], st.vel_body[1]
        odom.twist.twist.angular.z = st.rpy_vel[2] 
        self.odom_pub.publish(odom)

        if self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id = stamp, self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = x, y
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = q
            self.tf_br.sendTransform(t)

        self.basic_state_pub.publish(Int32(data=int(st.basic_state)))

        if st.ultrasound is not None:
            for pub, dist, frame in (
                (self.us_front_pub, st.ultrasound[0], "ultrasound_front"),
                (self.us_back_pub, st.ultrasound[1], "ultrasound_back"),
            ):
                r = Range()
                r.header.stamp, r.header.frame_id = stamp, frame
                r.radiation_type = Range.ULTRASOUND
                r.field_of_view = self.us_fov
                r.min_range, r.max_range = 0.05, self.us_max
                r.range = float(dist)
                pub.publish(r)

    # ---------------------------------------------------------- shutdown
    def destroy_node(self):
        self._running = False
        try:
            self.stop_robot()
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    # NO signal handler: Ctrl+C raises KeyboardInterrupt while the context is
    # still valid, so destroy_node() can still send the final zero velocity.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Lite3Driver()
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
