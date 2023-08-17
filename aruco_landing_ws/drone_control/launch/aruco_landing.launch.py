import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_landing_launch = Node(
        package='drone_control',
        executable='aruco_landing'
    )

    return LaunchDescription([
        aruco_landing_launch
    ])
