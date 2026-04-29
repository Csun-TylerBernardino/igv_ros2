from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zed2i'
        }.items()
    )

    urdf_path = os.path.join(
        get_package_share_directory('igv_description'),
        'urdf',
        'igv.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    behavior_manager = Node(
        package='igv_behavior',
        executable='behavior_manager',
        name='behavior_manager',
        output='screen'
    )

    odom_stub = Node(
        package='igv_localization',
        executable='odom_stub',
        name='odom_stub',
        output='screen'
    )

    cmd_vel_manager = Node(
        package='igv_control',
        executable='cmd_vel_manager',
        name='cmd_vel_manager',
        output='screen'
    )

    segformer_lane_node = Node(
        package='igv_perception',
        executable='segformer_lane_node',
        name='segformer_lane_node',
        output='screen'
    )

    obstacle_detector_node = Node(
        package='igv_perception',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[{
            'min_x': 0.65,
            'max_x': 1.40,
            'min_y': -0.30,
            'max_y': 0.30,
            'min_z': 0.12,
            'max_z': 0.75,
            'min_points': 50,
        }]
    )

    rviz_config = os.path.join(
        get_package_share_directory('igv_bringup'),
        'rviz',
        'igv_default.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        zed_launch,
        robot_state_publisher,
        behavior_manager,
        odom_stub,
        cmd_vel_manager,
        segformer_lane_node,
        obstacle_detector_node,
        rviz
    ])
