from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('usb_cam'),
                         'launch/launch.py')
    )
    )

    embedded_aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_aruco'),
                         'launch/embedded_aruco_node.launch.py')
        )
    )

    drone_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_urdf'),
                            'launch.py')
        )
    )

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_bringup'),
                         'launch/map.launch.py')
    )
    )

    aruco_listener_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_bringup'),
                         'launch/aruco_listener.launch.py')
    )
    )
    
    ld.add_action(usb_cam_launch)
    ld.add_action(embedded_aruco_node)
    ld.add_action(drone_urdf_launch)
    ld.add_action(map_launch)
    ld.add_action(aruco_listener_launch)

    return ld
