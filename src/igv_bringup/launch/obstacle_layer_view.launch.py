import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.expanduser(
        '~/ros2_ws/src/igv_bringup/config/obstacle_layer_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='local_costmap',
            name='local_costmap',
            output='screen',
            parameters=[params_file],
            emulate_tty=True
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_local_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['local_costmap/local_costmap'],
                'bond_timeout': 0.0
            }]
        ),
    ])
