"""Shared main() for the workshop nodes.

Ctrl+C handling: rclpy's default signal handler shuts the context down
*before* our ``finally`` block runs, so publishing a last zero Twist would
fail. With SignalHandlerOptions.NO, Ctrl+C is a plain KeyboardInterrupt and
the context is still valid when we send the stop command.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


def run(node_cls, args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # stop the robot / turtle on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
