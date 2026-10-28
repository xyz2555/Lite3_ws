"""go_to_point (+ optional fake robot, + optional twist_mux).

Real robot, simplest (no mux; do NOT run teleop at the same time):
  ros2 launch lite3_bringup bringup.launch.py                 # driver + guard
  ros2 launch lite3_navigation goto.launch.py

Real robot with teleop priority (go_to_point -> /cmd_vel_auto, teleop -> /cmd_vel_teleop):
  ros2 launch lite3_navigation goto.launch.py use_mux:=true

Without hardware (fake robot publishes odom->base_link; do NOT run lite3_driver):
  ros2 launch lite3_navigation goto.launch.py sim:=true
  ros2 launch lite3_navigation goto.launch.py sim:=true obstacle_x:=1.5     # virtual obstacle
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("sim", default_value="false",
                              description="Start fake_lite3 instead of relying on the real driver"),
        DeclareLaunchArgument("use_mux", default_value="false",
                              description="Run twist_mux (teleop priority over go_to_point)"),
        DeclareLaunchArgument("require_standing", default_value="true"),
        DeclareLaunchArgument("obstacle_x", default_value="nan",
                              description="sim only: virtual obstacle x in odom (nan = none)"),
        DeclareLaunchArgument("obstacle_y", default_value="0.0"),
    ]
    use_mux = PythonExpression(["'", lc("use_mux"), "'.lower() == 'true'"])
    cmd_topic = PythonExpression(["'/cmd_vel_auto' if '", lc("use_mux"),
                                  "'.lower() == 'true' else '/cmd_vel'"])
    teleop_topic = PythonExpression(["'/cmd_vel_teleop' if '", lc("use_mux"),
                                     "'.lower() == 'true' else ''"])
    pkg = FindPackageShare("lite3_navigation")

    go_to_point = Node(
        package="lite3_navigation", executable="go_to_point", name="go_to_point",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg, "config", "go_to_point.yaml"]),
            {"cmd_topic": ParameterValue(cmd_topic, value_type=str),
             "teleop_topic": ParameterValue(teleop_topic, value_type=str),
             "require_standing": lc("require_standing")},
        ],
    )
    mux = Node(
        package="twist_mux", executable="twist_mux", name="twist_mux", output="screen",
        condition=IfCondition(use_mux),
        parameters=[PathJoinSubstitution([pkg, "config", "twist_mux.yaml"])],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )
    fake = Node(
        package="lite3_navigation", executable="fake_lite3", name="fake_lite3",
        output="screen", condition=IfCondition(lc("sim")),
        parameters=[{"cmd_topic": "/cmd_vel",
                     "obstacle_x": ParameterValue(lc("obstacle_x"), value_type=float),
                     "obstacle_y": ParameterValue(lc("obstacle_y"), value_type=float)}],
    )
    return LaunchDescription(args + [go_to_point, mux, fake])
