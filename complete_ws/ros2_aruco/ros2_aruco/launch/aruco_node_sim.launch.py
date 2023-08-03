import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params_sim_large = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters_sim_large.yaml'
        )

    aruco_node_sim_large = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_params_sim_large]
    )

    aruco_params_sim_small = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters_sim_small.yaml'
        )

    aruco_node_sim_small = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_params_sim_small]
    )

    return LaunchDescription([
        aruco_node_sim_large,
        aruco_node_sim_small
    ])
