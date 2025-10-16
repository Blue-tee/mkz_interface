from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('mkz_interface'), 'config', 'topics.yaml'])

    return LaunchDescription([
        Node(
            package='mkz_interface',
            executable='autoware_to_dbw_can.py',
            name='autoware_to_dbw_can',
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package='mkz_interface',
            executable='dbw_can_to_autoware.py',
            name='dbw_can_to_autoware',
            output='screen',
            parameters=[cfg],
        ),
    ])

