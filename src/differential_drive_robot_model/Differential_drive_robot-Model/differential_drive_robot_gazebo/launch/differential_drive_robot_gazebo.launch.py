"""
differential_drive_robot_gazebo.launch.py
------------------------------------------
Top-level convenience launcher: custom world + robot.
Delegates to gazebo_world_visualize.launch.py.

Usage:
  ros2 launch differential_drive_robot_gazebo differential_drive_robot_gazebo.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

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

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('differential_drive_robot_gazebo'),
                'launch',
                'gazebo_world_visualize.launch.py',
            ])
        ),
        launch_arguments={
            'world':   LaunchConfiguration('world'),
            'z_pose':  LaunchConfiguration('z_pose'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        z_pose_arg,
        gazebo_world,
    ])
