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

# With the default DDS implementation, it was impossible to run more than about a dozen
# ROS nodes. After that, new nodes would fail with the error "Failed to find a free
# participant index for domain 0", even though the actual limit should be much higher [1].
# Switching to Fast-DDS made this issue disappear.
#
# [1] https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html#participant-constraints
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
