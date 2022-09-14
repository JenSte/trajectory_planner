#!/bin/bash

set -eu

ROS_VERSION=$1

echo "Installing ROS '$ROS_VERSION'"

curl \
    -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    >/etc/apt/sources.list.d/ros2.list

export DEBIAN_FRONTEND=noninteractive

PACKAGES="
    python3-colcon-common-extensions
    ros-${ROS_VERSION}-desktop
    ros-${ROS_VERSION}-gazebo-dev
    ros-${ROS_VERSION}-gazebo-msgs
    ros-${ROS_VERSION}-gazebo-plugins
    ros-${ROS_VERSION}-gazebo-ros
    ros-${ROS_VERSION}-nav2-bringup
    ros-${ROS_VERSION}-nav2-core
    ros-${ROS_VERSION}-nav2-costmap-2d
    ros-${ROS_VERSION}-nav2-util
    ros-${ROS_VERSION}-tf-transformations
    ros-${ROS_VERSION}-xacro
"

if [ "ROS_VERSION" = "galactic" ]; then
    PACKAGES="$PACKAGES ros-${ROS_VERSION}-rqt-tf-tree"
fi;

apt-get update
apt-get install --yes -o APT::Update::Error-Mode=any $PACKAGES

# After the 'exec ...' in our entrypoint script the bash completion is somehow broken,
# restore it for the 'ros2' command.
echo "source /opt/ros/${ROS_VERSION}/share/ros2cli/environment/ros2-argcomplete.bash" >> ~/.bashrc

# With the default DDS implementation, it was impossible to run more than about a dozen
# ROS nodes. After that, new nodes would fail with the error "Failed to find a free
# participant index for domain 0", even though the actual limit should be much higher [1].
# Switching to Fast-DDS made this issue disappear.
#
# [1] https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html#participant-constraints
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# Add some often used commands to the bash history.
echo "colcon build" >> ~/.bash_history
echo "colcon build && ./build/trajectory_planner/trajectory_planner_test" >> ~/.bash_history
echo "ros2 launch trajectory_planner_tools standalone.py" >> ~/.bash_history
echo "nice -n 19 ros2 launch trajectory_planner_tools rviz.py" >> ~/.bash_history
echo "ros2 launch trajectory_planner_tools utils.py" >> ~/.bash_history
echo "ros2 run trajectory_planner_tools client.py big" >> ~/.bash_history
