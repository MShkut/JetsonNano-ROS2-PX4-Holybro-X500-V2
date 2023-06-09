These are instructions for setting up the PX4-6C Flight controller serial communications with the Jetson Nano over the GPIO pins 6,8,10. The Jetson nano runs a custom ubuntu kernel to operate on Ubuntu 20.04, this is to allow the installation of ROS2 Foxy, which is necessary for the desired autonomous control later in the project. 

## Step 1

1. Download balena etcher - https://etcher.balena.io/
2. Download the custom Ubuntu 20.04 image here - https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768
3. Flash the Ubunutu 20.04 image onto your microSD card (We are using a 128GB microSD which is overkill but the higher read/write speeds are needed)
4. Insert the microSD card into the Jetson Nano following all the standard prompts to set up the OS
5. Run `sudo apt update` and `sudo apt upgrade`

## Step 2

In order to allow for serial communications to take place between the PX4-6C and the Jetson Nano's Serial Port the serial monitor which is enabled by default on the Jetson Nano must be disabled. This is called nvgetty. Run the commands below to do this. 

```
systemctl stop nvgetty
systemctl disable nvgetty
udevadm trigger
#you may want to reboot instead
```

## Step 3
 
Install Ros2 Foxy. Find the full instructions here - https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html . For ease of this guide I have copied the instructions below. 

1. 
Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.                     
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

2. 
Setup Sources. You will need to add the ROS 2 apt repository to your system.

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
3. 
Install ROS2 Packages & add to your .bashrc
```
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-dev-tools
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
#run source ~/.bashrc or open a new terminal (only needed for first time after entering above command)
```

