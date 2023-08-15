import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_listener_small = Node(
        package='drone_bringup',
        executable='aruco_listener_small'
    )

    return LaunchDescription([
        aruco_listener_small
    ])
