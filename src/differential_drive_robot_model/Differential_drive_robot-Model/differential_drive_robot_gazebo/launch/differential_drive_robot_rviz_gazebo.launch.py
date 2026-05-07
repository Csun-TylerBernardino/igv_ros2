"""
differential_drive_robot_rviz_gazebo.launch.py
-----------------------------------------------
Launches Gazebo Classic (with the custom world) AND RViz2 side-by-side.
Useful for watching the robot in simulation while also seeing the TF tree
and sensor data in RViz.

Usage:
  ros2 launch differential_drive_robot_gazebo differential_drive_robot_rviz_gazebo.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── Arguments ─────────────────────────────────────────────────────────
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

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            FindPackageShare('differential_drive_robot_description'),
            'rviz',
            'urdf.rviz',
        ]),
        description='Absolute path to the RViz2 configuration file.',
    )

    # ── Gazebo + robot (via world visualize sub-launch) ───────────────────
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('differential_drive_robot_gazebo'),
                'launch',
                'gazebo_world_visualize.launch.py',
            ])
        ),
        launch_arguments={
            'world':  LaunchConfiguration('world'),
            'z_pose': LaunchConfiguration('z_pose'),
        }.items(),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        z_pose_arg,
        rviz_config_arg,
        gazebo_world,
        rviz,
    ])
