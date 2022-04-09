# colcon build; ros2 launch trajectory_planner standalone.py

# Launch only the planner, not the complete navigation stack.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.actions


def generate_launch_description():

    package_dir = get_package_share_directory("trajectory_planner")
    map_dir = os.path.join(package_dir, "map")
    config_dir = os.path.join(package_dir, "config")

    nav2_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_dir, "launch")

    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable(
                "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
            ),
            launch_ros.actions.Node(
                name="planner_server",
                package="nav2_planner",
                executable="planner_server",
                output="screen",
                parameters=[os.path.join(config_dir, "standalone.yaml")],
            ),
            launch_ros.actions.Node(
                name="lifecycle_manager",
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[
                    {"autostart": True},
                    {"node_names": ["map_server", "planner_server"]},
                ]
            ),
            launch_ros.actions.Node(
                name="static_transform_map_base_link",
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=6 * ["0.0"] + ["map", "base_link"],
            ),
            launch_ros.actions.Node(
                name="map_server",
                package="nav2_map_server",
                executable="map_server",
                output="screen",
                # arguments=["--ros-args", "--log-level", "debug"],
                parameters=[
                    {"yaml_filename": os.path.join(map_dir, "map.yaml")},
                ],
            ),
        ]
    )
