import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('bouncing_ball_catcher')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    camera_node = Node(
        package='bouncing_ball_catcher',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[params_file],
    )

    game_node = Node(
        package='bouncing_ball_catcher',
        executable='game_node',
        name='game_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        camera_node,
        game_node,
    ])