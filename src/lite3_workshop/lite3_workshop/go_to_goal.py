"""Lab 3: subscriber + closed loop. The turtle drives itself to (x, y).

Change the goal while running:
  ros2 param set /go_to_goal x 2.0
Pause / resume (service from the "Service" slide):
  ros2 service call /go_to_goal/enable std_srvs/srv/SetBool "{data: false}"
"""

import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool
from turtlesim.msg import Pose

from lite3_workshop.common import run


class GoToGoal(Node):
    def __init__(self):
        super().__init__("go_to_goal")
        self.declare_parameter("x", 8.0)
        self.declare_parameter("y", 8.0)
        self.declare_parameter("k_turn", 4.0)
        self.pose = None
        self.enabled = True
        self.create_subscription(Pose, "/turtle1/pose", self.on_pose, 10)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_service(SetBool, "~/enable", self.on_enable)
        self.create_timer(0.05, self.control)  # 20 Hz

    def on_pose(self, msg):
        self.pose = msg                         # just store it, stay fast

    def on_enable(self, req, res):
        self.enabled = req.data
        res.success = True
        res.message = "on" if req.data else "off"
        return res

    def control(self):
        if self.pose is None:
            return
        cmd = Twist()
        if self.enabled:
            gx = self.get_parameter("x").value
            gy = self.get_parameter("y").value
            k = self.get_parameter("k_turn").value
            dx, dy = gx - self.pose.x, gy - self.pose.y
            dist = math.hypot(dx, dy)
            err = math.atan2(dy, dx) - self.pose.theta
            err = math.atan2(math.sin(err), math.cos(err))  # wrap to -pi..pi
            if dist > 0.05:
                cmd.angular.z = k * err
                if abs(err) < 0.5:
                    cmd.linear.x = min(1.5 * dist, 2.0)
        self.pub.publish(cmd)


def main(args=None):
    run(GoToGoal, args)


if __name__ == "__main__":
    main()
