#! /bin/bash
# Referece: https://github.com/csiro-robotics/dynamixel_interface/blob/master/readme.md

echo "╔══╣ Install: Dynamixel Hardware (STARTING) ╠══╗"

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-lifecycle \
    ros-$ROS_DISTRO-lifecycle-msgs \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-pluginlib \
    ros-$ROS_DISTRO-dynamixel-workbench-toolbox

# Add rules for Dynamixel
wget https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules
sudo mv ./99-dynamixel-workbench-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Serial Access
sudo usermod -a -G dialout $USER

echo "╚══╣ Install: Dynamixel Hardware (FINISHED) ╠══╝"
