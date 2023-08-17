import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    single_sim_aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'single_sim_aruco_parameters.yaml'
        )

    single_aruco_node = Node(
        package='ros2_aruco',
        executable='single_aruco_node',
        parameters=[single_sim_aruco_params]
    )

    return LaunchDescription([
        single_aruco_node
    ])
