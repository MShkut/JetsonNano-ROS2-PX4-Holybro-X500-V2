import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    my_robot_sim_controller_launch = Node(
        package='drone_control',
        executable='my_robot_sim_controller'
    )

    return LaunchDescription([
        my_robot_sim_controller_launch
    ])
