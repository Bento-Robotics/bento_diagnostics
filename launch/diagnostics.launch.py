from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')

    bento_diagnostics = Node(
        package='bento_diagnostics',
        executable='bento_diagnostics_node',
        name='bento_diagnostics',
        parameters=[
            {"publish_rate": 1000},
            {"config_file": PathJoinSubstitution([FindPackageShare('bento_diagnostics'), 'resource', 'diagnostics_config.yaml'])},
        ],
        output='screen',
        emulate_tty=True,
        namespace=robot_namespace,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='bento',
            description='set namespace for robot nodes'
        ),
        bento_diagnostics,
    ])
