# Trajectory Planner

A global planner for mobile robots that takes the vehicle's motion model into account.

## Mode of Operation

When searching in a higher-dimensional search space that takes the robot's speeds
(linear and rotational) and accelerations into account, the search space quickly
becomes so big that the search is not feasible any more.
To get around this, a search taking only the robots position and orientation is
done first, and the result of this first search is then inflated to come up with a
smaller search space for the higher-dimensional search.

## Status

- [x] Three-dimensional cost map
- [x] Three-dimensional planner
- [X] Higher-dimensional planner

# Usage

The planner is implemented as a plug-in for the
[ROS2 navigation stack](https://navigation.ros.org/).
After cloning this repository into a ROS2 workspace, it has to be compiled using the
colcon/ament built tools first.

If no "native" ROS2 installation is available, this repository also contains a set
of scripts that make it possible to run everything inside a Docker container.
First, call the script that builds an image with all prerequisites installed:
```
$ ./docker/build-ros-image.sh
[...]
Successfully tagged localhost/trajectory-planner-build-image:latest
```

Then, the container can be started as follows:
```
$ ./docker/run-ros-container.sh
sourcing ros installation
root@ros:/opt/ws#
```
Please note that the container is stopped and removed as soon as this first shell
exits. (However the workspace is not removed when the container stops, it is stored
in the directory `docker/workspace` and mounted as a volume.)

Additional shells in the container can be started with the
`docker/exec-ros-image.sh` script.
The first shell created by `docker/run-ros-container.sh` only sources the ROS2
installation (because the user's workspace is not already built), and should be used
to built the trajectory planner (otherwise the built tool will generate warnings),
while the additional shells source the new workspace and can be used to start the
planner.

Build the planner plugin:
```
root@ros:/opt/ws# colcon build
Starting >>> trajectory_planner
[Processing: trajectory_planner]
Finished <<< trajectory_planner [xy.zs]

Summary: 1 package finished [xy.zs]
```

Then run the planner (in a minimal configuration that only starts the parts of the
navigation stack that are necessary) by using the included launch-file in a shell
that has the freshly compiled workspace sourced:
```
$ ./docker/exec-ros-image.sh
sourcing workspace
root@ros:/opt/ws# ros2 launch trajectory_planner_tools standalone.py
```
The navigation stack will start up and is ready when the trajectory planner logs
the message `[planner_server.trajectory_planner]: Updated planner is ready.`.
The first start of the planner for a given map may take a while (depending on the
size of the map), as the planner creates the three-dimensional cost maps it uses.
These costs are written to disk and re-used if the same map and configuration
parameters are used again, so subsequent starts are much quicker.

A script is included that triggers the planning of a path:
```
$ ./docker/exec-ros-image.sh
sourcing workspace
root@ros:/opt/ws# ros2 run trajectory_planner_tools client.py big
[INFO] [] [compute_path_action_client]: Goal was accepted.
[INFO] [] [compute_path_action_client]: Path: ...
```

Two additional launch files are included in this repository:
* `utils.py`: Start additional nodes handy during development that create a plot
  containing debug information and that publish an animated footprint polygon when
  a path is received.
* `rviz.py`: Start rviz2 with all topics of interrest already added to the view.

# License

This project is licensed under the MIT license.
