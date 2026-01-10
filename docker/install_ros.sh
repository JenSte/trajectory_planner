#!/bin/bash

set -eu

ROS_VERSION=$1

echo "Installing ROS '$ROS_VERSION'"

# Both the ROS1 and ROS2 packages are signed with the same key.
curl \
    -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Configure APT, select packages.
if [ "$ROS_VERSION" = "melodic" ] || [ "$ROS_VERSION" = "noetic" ]; then
    # ROS1

    REPOSITORY=http://packages.ros.org/ros/ubuntu

    PACKAGES="
        libdxflib-dev
        libgoogle-perftools-dev
        ros-${ROS_VERSION}-costmap-2d
        ros-${ROS_VERSION}-desktop-full
        ros-${ROS_VERSION}-dwa-local-planner
        ros-${ROS_VERSION}-fake-localization
        ros-${ROS_VERSION}-grid-map-ros
        ros-${ROS_VERSION}-map-server
        ros-${ROS_VERSION}-move-base
        ros-${ROS_VERSION}-nav-core
    "

    python3 -m pip install transforms3d
else
    # ROS2

    REPOSITORY=http://packages.ros.org/ros2/ubuntu

    PACKAGES="
        python3-colcon-common-extensions
        python3-transforms3d
        ros-${ROS_VERSION}-desktop
        ros-${ROS_VERSION}-gazebo-msgs
        ros-${ROS_VERSION}-nav2-bringup
        ros-${ROS_VERSION}-nav2-core
        ros-${ROS_VERSION}-nav2-costmap-2d
        ros-${ROS_VERSION}-nav2-util
        ros-${ROS_VERSION}-tf-transformations
        ros-${ROS_VERSION}-xacro
    "

    if [ "ROS_VERSION" = "galactic" ]; then
        PACKAGES="$PACKAGES ros-${ROS_VERSION}-rqt-tf-tree"
    fi
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] $REPOSITORY $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
    >/etc/apt/sources.list.d/ros.list

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install --yes -o APT::Update::Error-Mode=any $PACKAGES

# Post-install stuff, prepare environment.
if [ "$ROS_VERSION" = "melodic" ] || [ "$ROS_VERSION" = "noetic" ]; then
    # ROS1
    # Add some often used commands to the bash history.
    echo "catkin_make" >> ~/.bash_history
    echo "roslaunch trajectory_planner_tools static.launch" >> ~/.bash_history
    echo "roslaunch trajectory_planner_tools simulation.launch" >> ~/.bash_history
    echo "rviz -d /opt/ws/src/trajectory_planner_tools_ros1/config/navigation.rviz" >> ~/.bash_history
else
    # ROS2

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
fi
