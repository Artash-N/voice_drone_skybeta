from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('voice_drone')
    cfg = PathJoinSubstitution([pkg_dir, 'config', 'voice_drone.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('cfg', default_value=cfg),
        Node(
            package='voice_drone',
            executable='console_node',
            name='console_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='intent_node',
            name='intent_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='oak_node',
            name='oak_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='detect_node',
            name='detect_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='telemetry_node',
            name='telemetry_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='safe_bridge_node',
            name='safe_bridge_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='fake_drone_node',
            name='fake_drone_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
        Node(
            package='voice_drone',
            executable='status_node',
            name='status_node',
            output='screen',
            parameters=[LaunchConfiguration('cfg')],
        ),
    ])
