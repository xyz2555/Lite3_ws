"""turtlesim + go_to_goal in one command (slide "Launch file")."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="turtlesim", executable="turtlesim_node", name="sim"),
        Node(package="lite3_workshop", executable="go_to_goal", name="go_to_goal",
             output="screen", parameters=[{"x": 2.0, "y": 8.0}]),
    ])
