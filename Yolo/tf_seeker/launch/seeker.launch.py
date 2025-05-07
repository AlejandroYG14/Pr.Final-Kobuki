import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tf_seeker_dir = get_package_share_directory('tf_seeker')

    tf_publisher_cmd = Node(
        package='tf_seeker',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen'
    )

    seeker_cmd = Node(
        package='tf_seeker',
        executable='seeker',
        name='seeker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()

    ld.add_action(tf_publisher_cmd)
    ld.add_action(seeker_cmd)

    return ld
