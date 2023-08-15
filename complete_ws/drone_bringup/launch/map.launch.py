import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    map = Node(
        package='drone_bringup',
        executable='map'
    )

    return LaunchDescription([
        map
    ])
