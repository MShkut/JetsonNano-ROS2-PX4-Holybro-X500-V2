These are instructions for setting up the PX4-6C Flight controller serial communications with the Jetson Nano over the GPIO pins 6,8,10. The Jetson nano runs a custom ubuntu kernel to operate on Ubuntu 20.04, this is to allow the installation of ROS2 Foxy, which is necessary for the desired autonomous control later in the project. Many commands are taken from https://docs.px4.io/v1.13/en/ros/ros2_comm.html, however I have simplified them for this specific application, including only what's necessary. 

## An overview of all the components used and on which devices throughout this process

Jetson Nano (Onboard Computer):
- Disable nvgetty
- Install ROS2 Foxy
- Install JDK 13
- Install Fast-RTPS-Gen
- Add usermod permissions
- Create ROS2 Workspace with ros_com & ros_msg

Your Personal Computer
- Install Balena Etcher
- Download Custom Ubuntu 20.04 Kernel & Flash to MicroSD
- Install PX4 Autopilot
- Install QGround Control
- Make Custom RTPS V6C Firmware

PX4-6C Flight Controller
- Flash Custom RTPS V6C firmware
- Enable RTPS on TELEM2
- Set Baud rate to 3000000

## Step 1 - Flash Ubuntu 20.04 OS

1. Download balena etcher - https://etcher.balena.io/
2. Download the custom Ubuntu 20.04 image here - https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768
3. Flash the Ubunutu 20.04 image onto your microSD card (We are using a 128GB microSD which is overkill but the higher read/write speeds are needed)
4. Insert the microSD card into the Jetson Nano following all the standard prompts to set up the OS
5. Run `sudo apt update` and `sudo apt upgrade`

## Step 2 - Disable nvgetty

In order to allow for serial communications to take place between the PX4-6C and the Jetson Nano's Serial Port the serial monitor which is enabled by default on the Jetson Nano must be disabled. This is called nvgetty. Run the commands below to do this. 

```
systemctl stop nvgetty
systemctl disable nvgetty
udevadm trigger
#you may want to reboot instead
```

## Step 3 - Install ROS2 Foxy
 
Install Ros2 Foxy. Find the full instructions here - https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html . For ease of this guide I have copied the instructions below. 

1. Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.                     
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

2. Setup Sources. You will need to add the ROS 2 apt repository to your system.

First ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
3. Install ROS2 Packages & add to your .bashrc
```
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
#run source ~/.bashrc or open a new terminal (only needed for first time after entering above command)
```

4. Ensure dependencies and need packages are installed
```
sudo apt install python3-colcon-common-extensions
sudo apt install ros-foxy-eigen3-cmake-module
sudo apt install python3-pip
sudo pip3 install -U empy pyros-genmsg setuptools 
```
if you get an error saying that test resources need to be installed then run
```
sudo apt-get install -y python3-testresources
```
and re-run the last command above in 4.

## Step 4 - Create firmware with microRTPS
While ROS2 Foxy is loading on the Jetson Nano, move over to your laptop/PC and install the PX4 Autopilot release 1.13, which will allow us to build the custom firmware needed to run microRTPS on our PX4-6C.

1. Download the PX4 Autopilot software release 1.13 and install it
```
git clone -b release/1.13 https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
2. Restart your computer as the guide suggests

3. Copy the rtps firmware tempate from v5 to v6c and edit it to disable dds paramaters
```
cd ~/PX4-Autopilot/boards/px4/fmu-v5
cp rtps.px4board /PX4-Autopilot/boards/px4/fmu-v6c
cd ..
cd fmu-v6c
gedit rtps.px4board #install gedit if you haven't already 
##Ensure the file has the following paramaters enabled/disabled
CONFIG_DRIVERS_HEATER=n
CONFIG_DRIVERS_OSD=n
CONFIG_MODULES_MICRODDS_CLIENT=n
CONFIG_MODULES_MICRORTPS_BRIDGE=y
```
save and close the file

4. Make the firmware
```
cd PX4-Autopilot/
make px4_fmu-v6c_rtps
```
## Step 5 - Download QGroundControl & Install the custom firmware on the PX4-6C (on your laptop/PC)
1.
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```
Restart your computer

2. Download the app image from here: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu

3. Install and run QGroundControl
```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
```

4. Connect your flight controller by USB, go into vehicle setup, then firmware. Click advanced settings, custom firware, and then find the file you created in the previous step. This should be in the ~/PX4-Autopilot/build/px4_fmu-v6c_rtps directory and the file will be called px4_fmu-v6c_rtps.px4. Select this and load it onto the board

## Step 5 - Set the firmware paramaters and set up the drone
1. Follow the standard set-up instructions for configuring your drone, not covered here as it's well documented on the PX4 website.

2.  Open the firmware settings go to MAV_2_CONFIG and ensure it is set to disabled, reboot the vehicle

3. Open the firmware settings again, type RTPS_Config and set it to TELEM2, reboot the vehicle

4. Open the firware settings again, set SER_TEL2_BAUD to 3000000, reboot the vehicle

These are all the firmware settings related to the serial communications.

## Step 6 - Install Fast-RTPS-Gen on the Jetson nano
1. Install java 13
```
sudo apt-get install openjdk-13-jre
```

2. Install Fast-RTPS-Gen
```
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \ #you may need to hit enter if stuck on >
cd ~/Fast-RTPS-Gen/gradle/wrapper
gedit gradle-wrapper.properties
distributionUrl=https\://services.gradle.org/distributions/gradle-6.8.3-bin.zip
cd ~/Fast-RTPS-Gen 
./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install
```

## Step 7 - Install ros_com and ros_msg in a ROS2 workspace

1. create a directory
```
mkdir -p ~/px4_ros_com_ros2/src
cd ~/px4_ros_com_ros2/src
```

2. Clone the ROS 2 bridge packages px4_ros_com and px4_msgs to the /src directory (the release/1.13 branch must be cloned):
```
git clone -b release/1.13 https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com
git clone -b release/1.13 https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
```

3. Use the build_ros2_workspace.bash script to build the ROS 2 workspace (including px4_ros_com and px4_msgs).
```
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```

4. If the build was unsuccessful run source build_ros2_workspace.bash again and it should compile quickly and correctly. 

5. Add workspace to the directory
```
echo source ~/px4_ros_com_ros2/install/setup.bash' >> ~/.bashrc
```

## Step 8 - Test the connection is open and working 

1. Open a new terminal on the Nano and run
```
sudo usermod -a -G tty USERNAME
sudo usermod -a -G dialout USERNAME
```
Restart the jetson nano

2. Connect the PX4-6C to your computer and open QGroundControl. Click the Q -> Analyze Tools -> Mavlink console -> type 'micrortps_client status' to ensure it is running on the flight controller

4. Open a new terminal on the nano and run
```
micrortps_agent -d /dev/ttyTHS1 -b 3000000
```
You should see an output with publishers and subscribers listed

5. Open a new terminal and run
```
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

you should see similar data to
```
RECEIVED DATA FROM SENSOR COMBINED
================================
ts: 870938190
gyro_rad[0]: 0.00341645
gyro_rad[1]: 0.00626475
gyro_rad[2]: -0.000515705
gyro_integral_dt: 4739
accelerometer_timestamp_relative: 0
accelerometer_m_s2[0]: -0.273381
accelerometer_m_s2[1]: 0.0949186
accelerometer_m_s2[2]: -9.76044
accelerometer_integral_dt: 4739
```

**Note that PX4 Autopilot & QGroundControl do not need to be installed on the jetson nano for this application but do need to be installed on your computer for the necessary steps listed above
