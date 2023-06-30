import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    odom = Node(
        package='drone_bringup',
        executable='odom'
    )

    return LaunchDescription([
        odom
    ])
