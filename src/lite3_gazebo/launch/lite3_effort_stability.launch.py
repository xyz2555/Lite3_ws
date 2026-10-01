from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)

import os


# ==============================================================
# Gazebo Odometry Plugin
# ==============================================================

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

    # ==========================================================
    # Package paths
    # ==========================================================

    description_share = get_package_share_directory(
        "lite3_description"
    )

    gazebo_share = get_package_share_directory(
        "lite3_gazebo"
    )

    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )

    # ==========================================================
    # Files
    # ==========================================================

    urdf_file = os.path.join(
        description_share,
        "Lite3",
        "urdf",
        "Lite3_gazebo.urdf"
    )

    world_file = os.path.join(
        gazebo_share,
        "worlds",
        "flat.sdf"
    )

    # ==========================================================
    # Read URDF
    # ==========================================================

    with open(
        urdf_file,
        "r",
        encoding="utf-8"
    ) as f:

        robot_description = f.read()

    if "</robot>" not in robot_description:

        raise RuntimeError(
            "Invalid Lite3_gazebo.urdf"
        )

    # Add odometry plugin.
    robot_description = robot_description.replace(
        "</robot>",
        ODOMETRY_PLUGIN + "</robot>"
    )

    # ==========================================================
    # Gazebo
    #
    # IMPORTANT:
    # -r means Gazebo runs immediately.
    #
    # We DO NOT pause Gazebo because controller_manager
    # requires update cycles for controller startup/switching.
    # ==========================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py"
            )

        ),

        launch_arguments={

            "gz_args":
                f'-r -v 3 "{world_file}"'

        }.items(),

    )

    # ==========================================================
    # Robot State Publisher
    # ==========================================================

    robot_state_publisher = Node(

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

        ]

    )

    # ==========================================================
    # Gazebo -> ROS 2 bridge
    # ==========================================================

    bridge = Node(

        package="ros_gz_bridge",

        executable="parameter_bridge",

        arguments=[

            "/clock@rosgraph_msgs/msg/Clock"
            "[gz.msgs.Clock",

            "/lite3/odometry@nav_msgs/msg/Odometry"
            "[gz.msgs.Odometry",

        ],

        output="screen",

    )

    # ==========================================================
    # Spawn Lite3
    # ==========================================================

    spawn = Node(

        package="ros_gz_sim",

        executable="create",

        arguments=[

            "-name",
            "lite3",

            "-topic",
            "robot_description",

            "-x",
            "-0.50",

            "-y",
            "0.0",

            "-z",
            "0.35",

        ],

        output="screen",

    )

    # ==========================================================
    # Joint State Broadcaster
    # ==========================================================

    joint_state_broadcaster = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[

            "joint_state_broadcaster",

            "--controller-manager",
            "/controller_manager",

        ],

        output="screen",

    )

    # ==========================================================
    # Position Controller
    #
    # ACTIVE from the beginning.
    #
    # This keeps Lite3 standing while we prepare the effort
    # controller and stability controller.
    # ==========================================================

    position_controller = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[

            "lite3_position_controller",

            "--controller-manager",
            "/controller_manager",

        ],

        output="screen",

    )

    # ==========================================================
    # Effort Controller
    #
    # IMPORTANT:
    # Starts INACTIVE.
    #
    # The correct argument is:
    #
    #     --inactive
    #
    # NOT:
    #
    #     -- inactive
    # ==========================================================

    effort_controller = Node(

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

    # ==========================================================
    # Three-leg stability controller
    #
    # This node starts while the POSITION controller is active.
    #
    # It continuously calculates:
    #
    #     tau = G(q)
    #           + Kp(qd-q)
    #           + Kd(qdot_d-qdot)
    #
    # but the effort controller is still inactive.
    #
    # After manual position -> effort switch, the same torque
    # command is continuously available.
    # ==========================================================

    stability_controller = Node(

        package="lite3_gazebo",

        executable="gravity_only_controller",

        name="gravity_only_controller",

        output="screen",

        parameters=[

            {

                "use_sim_time":
                    True,

                # "start_experiment":
                #     False,

                "start_handover": False,
                
                "kp":
                    40.0,

                "kd":
                    1.5,

                # "publish_rate":
                #     500.0,

                # "body_shift_x":
                #     -0.010,

                # "body_shift_y":
                #     -0.010,

                # "body_shift_duration":
                #     4.0,

                # "shift_hold_duration":
                #     1.5,

                # "fl_lift_height":
                #     0.005,

                # "fl_lift_duration":
                #     2.5,

                # "fl_hold_duration":
                #     1.0,

                # "prepare_duration":
                #     2.0,

            }

        ]

    )

    # ==========================================================
    # Controller startup
    #
    # After spawn:
    #
    #   1. joint_state_broadcaster
    #   2. position controller
    #   3. effort controller inactive
    #
    # ==========================================================

    start_controllers = RegisterEventHandler(

        event_handler=OnProcessExit(

            target_action=spawn,

            on_exit=[

                TimerAction(

                    period=2.0,

                    actions=[

                        joint_state_broadcaster,

                        position_controller,

                        effort_controller,

                    ]

                )

            ]

        )

    )

    # ==========================================================
    # Start stability controller only AFTER
    # the POSITION controller has successfully started.
    #
    # This avoids a race condition where the stability node
    # starts before joint_states are available.
    # ==========================================================

    start_stability_controller = RegisterEventHandler(

        event_handler=OnProcessExit(

            target_action=position_controller,

            on_exit=[

                TimerAction(

                    period=1.0,

                    actions=[

                        stability_controller

                    ]

                )

            ]

        )

    )

    # ==========================================================
    # Launch Description
    # ==========================================================

    return LaunchDescription([

        gazebo,

        bridge,

        robot_state_publisher,

        spawn,

        start_controllers,

        start_stability_controller,

    ])