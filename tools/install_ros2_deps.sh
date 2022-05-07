#!/usr/bin/env bash

# This script is meant to be run for CI, otherwise it can break exisitng setups

set -x

DISTRO="$(lsb_release -sc)"
if [[ "$DISTRO" == "focal" ]]; then
    ROS_DISTRO="galactic"
else
    echo "ROS2 support only ubuntu focal"
    exit 1
fi

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y ros-$ROS_DISTRO-ros-base

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

sudo apt-get install -y ros-$ROS_DISTRO-vision-opencv ros-${ROS_DISTRO}-image-transport libyaml-cpp-dev
sudo apt-get install -y ros-${ROS_DISTRO}-tf2-sensor-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-mavros*
sudo apt-get install -y python3-pip python3-yaml python3-setuptools python3-colcon-common-extensions
