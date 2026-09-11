from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    description_share = get_package_share_directory(
        'lite3_description'
    )

    gazebo_share = get_package_share_directory(
        'lite3_gazebo'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    urdf_file = os.path.join(
        description_share,
        'Lite3',
        'urdf',
        'Lite3_gazebo.urdf'
    )

    world_file = os.path.join(
        gazebo_share,
        'worlds',
        'flat.sdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # --------------------------------------------------
    # Gazebo
    # --------------------------------------------------

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 "{world_file}"'
        }.items()
    )

    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ]
    )

    # --------------------------------------------------
    # Spawn robot
    # --------------------------------------------------

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lite3',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.40'
        ],
        output='screen'
    )

    # --------------------------------------------------
    # Controllers
    # --------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'lite3_position_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Start controllers after robot spawn
    controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[
                joint_state_broadcaster_spawner,
                position_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        gz_sim,
        rsp,
        spawn,
        controllers_after_spawn
    ])