## Drone Bringup
- Drone bringup contains the aruco listener, odometry transform (map), and general launch files
  - aruco listener: This program creates a transform from the aruco marker to map, which is the home/ takeoff position of the drone when launching. This is used for calculations between the position of        the drone and the position of the marker
  - map: This program is used to take in odometry data from the flight controller, rotate it to match the ROS2 frame convention, and reformat and publish it as a topic and through a transform
 
## Drone Control
- Drone control contains the landing algorithms for the landing processes along with some offboard testing programs
  - aruco landing: this program has the ability to publish updated parameters over a topic for the aruco node to update it, aka an embedded parameter, currently the embedded landing has been less accurate     than a single marker so this can be commented out and further tested later if necessary
  - sim aruco landing: this program is near identical to the aruco landing but as the embedded marker does not function as desired in the simulaiton environment the embedded part of this program has not       been included
 
## ROS2 Aruco
- ROS2 Aruco includes both the interfaces and aruco detection algorithms used during landing
    - Config contains the initial parameters (if using the embedded approach), all parameters listed in the aruco node files
    - embedded aruco node: contains the aruco node detection algorithm and also funtionality to update the parameters being used mid flight (triggered from the control program)
    - single aruco node: near identical to the embedded node, with functionality to update parameters removed.

## The Rest
- Drone URDF contains the drone urdf file to visualization in RVIZ and the correct placement of the devices for accurate localiation, RVIZ can be enabled or disabled in the launch file
- PX4 ros com and PX4 msgs are slightly modified versions of the standard autopilot stack
- USB cam & image pipline are the standard libraries used to send webcam information into ROS2

