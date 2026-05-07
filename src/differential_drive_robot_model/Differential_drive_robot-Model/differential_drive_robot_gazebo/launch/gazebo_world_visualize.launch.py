"""
gazebo_world_visualize.launch.py
---------------------------------
Launches Gazebo Classic with the package's custom world file and spawns
the robot.  Called by differential_drive_robot_gazebo.launch.py.

Usage (direct):
  ros2 launch differential_drive_robot_gazebo gazebo_world_visualize.launch.py
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

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('differential_drive_robot_gazebo'),
            'world',
            'world.world',
        ]),
        description='Absolute path to the Gazebo world file.',
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.12',
        description='Spawn height of the robot origin above the ground plane (metres).',
    )

    # ── Robot description ─────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # ── Gazebo Classic with custom world ──────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
        }.items(),
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
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
        world_arg,
        z_pose_arg,
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
    ])
