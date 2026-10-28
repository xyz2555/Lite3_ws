"""Lite3 bringup: driver + obstacle guard (+ optional RealSense, waypoints).

    cmd_vel --> obstacle_guard --> /lite3/cmd_vel_safe --> lite3_driver --> UDP 43893

Examples
  ros2 launch lite3_bringup bringup.launch.py
  ros2 launch lite3_bringup bringup.launch.py robot_ip:=192.168.2.1
  ros2 launch lite3_bringup bringup.launch.py use_depth:=true launch_realsense:=true
  ros2 launch lite3_bringup bringup.launch.py use_guard:=false        # no safety layer
  ros2 launch lite3_bringup bringup.launch.py run_waypoints:=true \\
      waypoints_file:=/path/to/plan.json
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.120",
                              description="Lite3 motion host IP (or 192.168.2.1)"),
        DeclareLaunchArgument("local_port", default_value="43897",
                              description="UDP port the robot reports to (network.toml)"),
        DeclareLaunchArgument("use_guard", default_value="true",
                              description="Insert obstacle_guard between cmd_vel and the driver"),
        DeclareLaunchArgument("use_ultrasound", default_value="true",
                              description="Guard source: built-in ultrasound"),
        DeclareLaunchArgument("use_depth", default_value="false",
                              description="Guard source: forward depth camera"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/camera/depth/image_rect_raw"),
        DeclareLaunchArgument("stop_distance", default_value="0.5"),
        DeclareLaunchArgument("slow_distance", default_value="1.0"),
        DeclareLaunchArgument("launch_realsense", default_value="false",
                              description="Also start realsense2_camera (depth only)"),
        DeclareLaunchArgument("run_waypoints", default_value="false"),
        DeclareLaunchArgument("waypoints_file", default_value=""),
    ]

    driver_cmd_topic = PythonExpression([
        "'/lite3/cmd_vel_safe' if '", lc("use_guard"), "'.lower() == 'true' else '/cmd_vel'"])

    driver = Node(
        package="lite3_driver", executable="driver_node", name="lite3_driver",
        output="screen",
        parameters=[{
            "robot_ip": lc("robot_ip"),
            "robot_port": 43893,
            "local_port": lc("local_port"),
            "cmd_vel_topic": driver_cmd_topic,
        }],
    )

    guard = Node(
        package="lite3_safety", executable="obstacle_guard", name="obstacle_guard",
        output="screen", condition=IfCondition(lc("use_guard")),
        parameters=[
            PathJoinSubstitution([FindPackageShare("lite3_safety"), "config",
                                  "obstacle_guard.yaml"]),
            {
                "cmd_vel_in": "/cmd_vel",
                "cmd_vel_out": "/lite3/cmd_vel_safe",
                "use_ultrasound": lc("use_ultrasound"),
                "use_depth": lc("use_depth"),
                "depth_topic": lc("depth_topic"),
                "stop_distance": lc("stop_distance"),
                "slow_distance": lc("slow_distance"),
            },
        ],
    )

    # Approximate ultrasound mounting frames so Range shows up in RViz.
    # Measure your robot and adjust.
    us_front_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="us_front_tf",
        arguments=["--x", "0.30", "--z", "0.05", "--frame-id", "base_link",
                   "--child-frame-id", "ultrasound_front"])
    us_back_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="us_back_tf",
        arguments=["--x", "-0.30", "--z", "0.05", "--yaw", "3.14159265",
                   "--frame-id", "base_link", "--child-frame-id", "ultrasound_back"])

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
        launch_arguments={"enable_color": "false", "enable_depth": "true",
                          "depth_module.profile": "640x480x30"}.items(),
        condition=IfCondition(lc("launch_realsense")),
    )

    runner = TimerAction(period=2.0, actions=[Node(
        package="lite3_motion", executable="waypoint_runner",
        name="lite3_waypoint_runner", output="screen",
        parameters=[{"waypoints_file": lc("waypoints_file"),
                     "cmd_vel_topic": "/cmd_vel"}],
        condition=IfCondition(lc("run_waypoints")),
    )])

    return LaunchDescription(args + [driver, guard, us_front_tf, us_back_tf, realsense, runner])
