from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


ODOMETRY_PLUGIN = """
  <gazebo>
    <plugin
      filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>TORSO</robot_base_frame>
      <odom_publish_frequency>100</odom_publish_frequency>
      <odom_topic>/lite3/odometry</odom_topic>
      <dimensions>3</dimensions>
      <gaussian_noise>0.0</gaussian_noise>
    </plugin>
  </gazebo>
"""


def generate_launch_description():

    description_share = get_package_share_directory(
        "lite3_description"
    )

    gazebo_share = get_package_share_directory(
        "lite3_gazebo"
    )

    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    urdf_file = os.path.join(
        description_share,
        "Lite3",
        "urdf",
        "Lite3_gazebo.urdf",
    )

    world_file = os.path.join(
        gazebo_share,
        "worlds",
        "stair_one_step.sdf",
    )

    stair_controller_config = os.path.join(
        gazebo_share,
        "config",
        "stair_one_step.yaml",
    )

    with open(
        urdf_file,
        "r",
        encoding="utf-8"
    ) as f:
        robot_description = f.read()

    if "</robot>" not in robot_description:
        raise RuntimeError(
            "Invalid Lite3_gazebo.urdf: missing </robot>"
        )

    robot_description = robot_description.replace(
        "</robot>",
        ODOMETRY_PLUGIN + "</robot>"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args":
                f'-r -v 4 "{world_file}"',
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description":
                    robot_description,
                "use_sim_time":
                    True,
            }
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/lite3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "lite3",
            "-topic", "robot_description",
            "-x", "-0.55",
            "-y", "0.0",
            "-z", "0.345",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lite3_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lite3_effort_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    stair_controller = Node(
        package="lite3_gazebo",
        executable="single_step_stair_controller",
        name="single_step_stair_controller",
        output="screen",
        parameters=[
            stair_controller_config
        ],
    )

    controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        position_controller_spawner,
                        effort_controller_spawner,
                    ],
                )
            ],
        )
    )

    start_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        stair_controller
                    ],
                )
            ],
        )
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge,
        rsp,
        spawn,
        controllers_after_spawn,
        start_after_controllers,
    ])