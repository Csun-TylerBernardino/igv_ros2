from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    scale_linear_arg = DeclareLaunchArgument(
        'scale_linear',
        default_value='0.5',
        description='Default linear speed scale used by the keyboard teleop node.',
    )

    scale_angular_arg = DeclareLaunchArgument(
        'scale_angular',
        default_value='1.5',
        description='Default angular speed scale used by the keyboard teleop node.',
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic.',
    )

    teleop_node = Node(
        package='differential_drive_robot_control',
        executable='teleop',
        name='teleop',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'scale_linear': LaunchConfiguration('scale_linear'),
            'scale_angular': LaunchConfiguration('scale_angular'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }],
    )

    return LaunchDescription([
        scale_linear_arg,
        scale_angular_arg,
        cmd_vel_topic_arg,
        teleop_node,
    ])
