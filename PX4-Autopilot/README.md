PX4 Workspace with changes to:
- UORB topics sent over RTPS
- The empty world (which contains an aruco now at 5m, 5m, ~0.1m)
- The rtps firmware uploaded to the drone located in boards
- An edited Iris drone which now includes a camera which sends image topics over ROS2

I've only included the modified files in their respective directories

This is from release 1.13 of the PX4 Autopilot software: https://github.com/PX4/PX4-Autopilot/releases/tag/v1.13.3


Replace all files in the PX4-Autopilot folder with their respective counterparts in this repo.

Ensure you have built the aruco marker from the gazebo_models branch of the main repo for it to be loaded in the empty world. You may have to edit the empty world file to adjust the name of the aruco marker. 

To edit the UORB rtps messages being sent:

Step 1: 
```
cd ~/PX4-Autopilot/msg/tools
python3 uorb_to_ros_urtps_topics.py -i urtps_bridge_topics.yaml -o ~/px4_ros_com_ros2/src/px4_ros_com/templates/urtps_bridge_topics.yaml
```

Step 2: 
```
cd ~/PX4-Autopilot
make clean 
rm -rf build/
```

Step 3: 
```
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts 
source clean_all.bash
```

Step 4: 
```
colcon build
colcon build
```

Step 5(Simulation):
```
make px4_sitl_rtps gazebo_iris
```
to allow the drone to run in offboard mode you may need to change these parameters
```
param set COM_RCL_EXCEPT 4
param set NAV_DLL_ACT 0
param set NAV_RCL_ACT 0
```

Step 5(Harware):
```
make px4_fmu-v6c_rtps
```
Connect your flight controller by USB, go into vehicle setup, then firmware. Click advanced settings, custom firware, and then find the file you created. This should be in the ~/PX4-Autopilot/build/px4_fmu-v6c directory and the file will be called rtps.px4board. Select this and load it onto the board.
