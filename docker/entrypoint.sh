#!/bin/bash

if [ -f /opt/ws/install/setup.sh ]; then
    echo "sourcing workspace"
    source /opt/ws/install/setup.sh
else
    echo "sourcing ros installation"
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        source /opt/ros/galactic/setup.bash
    fi
fi

cd /opt/ws

exec $1
