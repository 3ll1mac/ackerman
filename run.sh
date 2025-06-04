# /bin/sh
# This exe will install ros2 for jazzy distribution, including ros_control and gazebo


# Install ros2
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-ros-base
source /opt/ros/jazzy/setup.bash

# Some useful packages
sudo apt install vim
sudo apt install xterm
sudo apt install libserial-dev 

#gazebo
sudo apt-get install ros-jazzy-ros-gz

#ros_control
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-gz-ros2-control-demos

