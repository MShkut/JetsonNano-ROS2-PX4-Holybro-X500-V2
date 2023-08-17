import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    initial_embedded_aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'initial_embedded_aruco_parameters.yaml'
        )

    embedded_aruco_node_launch = Node(
        package='ros2_aruco',
        executable='embedded_aruco_node',
        parameters=[initial_embedded_aruco_params]
    )

    return LaunchDescription([
        embedded_aruco_node_launch
    ])
