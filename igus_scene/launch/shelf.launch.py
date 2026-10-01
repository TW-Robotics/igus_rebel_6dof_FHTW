import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("igus_scene"), "config", "shelf.yaml")
    return LaunchDescription(
        [Node(package="igus_scene", executable="shelf_scene", parameters=[config], output="screen", respawn=True, respawn_delay=2.0)]
    )
