import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    land_launch = Node(
        package='drone_control',
        executable='land'
    )

    return LaunchDescription([
        land_launch
    ])
