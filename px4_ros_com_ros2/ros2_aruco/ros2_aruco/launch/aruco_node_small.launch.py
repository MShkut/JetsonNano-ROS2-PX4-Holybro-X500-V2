import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    aruco_small_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters_small.yaml'
        )

    aruco_node_small = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_small_params]
    )

    return LaunchDescription([
        aruco_node_small
    ])
