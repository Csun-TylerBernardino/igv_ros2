"""
gazebo_visualize.launch.py
--------------------------
Launches Gazebo Classic (Gazebo 11) with an empty world and spawns the
differential-drive robot.  This is the simplest entry point – no RViz,
no custom world file.

Usage:
  ros2 launch differential_drive_robot_gazebo gazebo_visualize.launch.py
  ros2 launch differential_drive_robot_gazebo gazebo_visualize.launch.py z_pose:=0.12
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── Arguments ─────────────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([
            FindPackageShare('differential_drive_robot_description'),
            'urdf',
            'differential_drive_robot.xacro',
        ]),
        description='Absolute path to the robot xacro file.',
    )

    # z_pose: spawn height above the ground plane.
    # wheel_radius = 0.1 m  →  robot origin (chassis link) sits at z = 0.1 m
    # when the wheels just touch the ground.  A tiny extra margin (0.12)
    # prevents interpenetration on the first physics tick and avoids the
    # robot bouncing/exploding at startup.
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.12',
        description='Spawn height of the robot origin above the ground plane (metres).',
    )

    # ── Robot description (xacro → URDF string) ───────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # ── Gazebo Classic ────────────────────────────────────────────────────
    # gazebo.launch.py from gazebo_ros starts both gzserver and gzclient.
    # No world argument → uses the default empty world.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    # Publishes /robot_description and static TF frames from the URDF.
    # use_sim_time=True is required so all nodes share Gazebo's clock.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── spawn_entity ──────────────────────────────────────────────────────
    # Reads the URDF from the /robot_description topic and asks Gazebo to
    # spawn it.  The TimerAction gives Gazebo time to finish loading before
    # the spawn request arrives.
    # NOTE: joint_state_publisher is NOT launched here.  The diff-drive
    # plugin (publish_wheel_tf=true) publishes wheel joint states directly,
    # so a separate joint_state_publisher would conflict with it.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'differential_drive_robot',
            '-z', LaunchConfiguration('z_pose'),
        ],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        z_pose_arg,
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
    ])
