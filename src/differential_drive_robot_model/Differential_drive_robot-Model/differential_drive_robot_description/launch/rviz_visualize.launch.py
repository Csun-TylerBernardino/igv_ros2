"""
rviz_visualize.launch.py
-------------------------
Standalone RViz2 visualiser – no Gazebo.  Useful for checking the URDF
model geometry and TF tree without starting a simulation.

Usage:
  ros2 launch differential_drive_robot_description rviz_visualize.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            FindPackageShare('differential_drive_robot_description'),
            'rviz',
            'urdf.rviz',
        ]),
        description='Absolute path to the RViz2 configuration file.',
    )

    # ── Robot description ─────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            LaunchConfiguration('model'),
        ]),
        value_type=str,
    )

    # ── joint_state_publisher ─────────────────────────────────────────────
    # Needed here (unlike the Gazebo launches) because there is no diff-drive
    # plugin to publish wheel joint states; the GUI slider widget lets you
    # rotate the wheels manually for URDF inspection.
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
