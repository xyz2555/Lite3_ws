"""Lab 2: first publisher. Drives turtlesim (or the Lite3) in a circle."""

from geometry_msgs.msg import Twist
from rclpy.node import Node

from lite3_workshop.common import run


class HelloPub(Node):
    def __init__(self):
        super().__init__("hello_pub")
        self.declare_parameter("topic", "/turtle1/cmd_vel")
        topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.pub.publish(msg)


def main(args=None):
    run(HelloPub, args)


if __name__ == "__main__":
    main()
