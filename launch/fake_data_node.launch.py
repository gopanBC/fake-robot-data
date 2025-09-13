from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fake_diffdrive'),
        'config',
        'config.yaml'
    )


    return LaunchDescription([
        Node(
        package='fake_diffdrive',
        executable='fake_diffdrive_node',
        name='fake_diffdrive_node',
        parameters=[config],
        output='screen'
        )
    ])