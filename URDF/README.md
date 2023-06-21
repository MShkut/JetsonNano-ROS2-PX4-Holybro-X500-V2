All files and workspaces used to import the URDF of the drone with webcam and flight controller into RVIZ2

ROS1 to ROS2 URDF tool: https://github.com/xiaoming-sun6/sw2urdf_ros2

SW2URDF tool: https://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly

## How to use these files

The test_urdf_tools_ws has been built for use with RVIZ2 (already converted using the tool listed above).

### 1. Download the test_urdf_tools_ws

### 2. Edit your bashrc to source the above ws using
```
gedit ~/.bashrc

# add to end of path
source ~/test_ws/install/local_setup.bash
```

### 3. download the holybro_drone package and unzip meshes into the directory it is currently in (delete the zip after)

### 4. Changing configuration
We must change the configuration variables in conversion_urdf_ros_2_ros2.py if we use this tool.

```
# Configuration variable

# This variable is the path to the solidworks output folder of urdf files
source_dir = '/home/sxm/Project/ros/test_urdf/'

# This variable is the package path of ros2
target_dir = '/home/sxm/Project/ros/test_urdf_tool_ws/src/test_urdf_tool/'

# This variable is the package-name of ros2
package_name = "test_urdf_tool"

# This variable is the solidworks output folder name
output_folder_name = "test_urdf"
```

After changing the configuration variables, we run the python file in a new terminal.
```
python conversion_urdf_ros_2_ros2.py
```
Then,We go into the workspace and build the code.

cd .. && colcon build && source install/setup.bash

### 5. Running launch file
Finally，We run the launch files.
```
ros2 launch drone_urdf launch.py 
```

## Troubleshooting
If the above steps did not work try following the ROS1 to ROS2 URDF tool instructions to build from scratch using the holybro_drone files given
