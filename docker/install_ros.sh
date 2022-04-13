#!/bin/bash

curl \
    -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    >/etc/apt/sources.list.d/ros2.list

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install --yes \
    python3-colcon-common-extensions \
    ros-galactic-desktop \
    ros-galactic-nav2-bringup \
    ros-galactic-nav2-core \
    ros-galactic-nav2-costmap-2d \
    ros-galactic-nav2-util

# After the 'exec ...' in our entrypoint script the bash completion is somehow broken,
# restore it for the 'ros2' command.
echo "source /opt/ros/galactic/share/ros2cli/environment/ros2-argcomplete.bash" >> ~/.bashrc
