#!/bin/bash

if [ -f /opt/ws/install/setup.sh ]; then
    echo "sourcing workspace"
    source /opt/ws/install/setup.sh
else
    echo "sourcing ros installation"
    source /opt/ros/galactic/setup.bash
fi

cd /opt/ws

exec $1
