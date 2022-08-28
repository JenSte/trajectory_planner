# Launch a Gazebo simulation using a world and robot model
# from the tuw_gazebo/tuw_gazebo_models packages, and start
# the nodes necessary to create a map of the world.
#
# - Start this launch file.
# - Use
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   to drive the robot around.
# - Start rviz and watch the map grow.
# - Use
#   ros2 run nav2_map_server map_saver_cli -f ...
#   to save the map.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions


def generate_launch_description():

    package_dir = get_package_share_directory("trajectory_planner_tools")
    config_dir = os.path.join(package_dir, "config")

    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    [
                        launch.substitutions.PathJoinSubstitution(
                            [
                                launch_ros.substitutions.FindPackageShare("tuw_gazebo"),
                                "launch",
                                "world.launch.py"
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "room": "cave",
                }.items(),
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    [
                        launch.substitutions.PathJoinSubstitution(
                            [
                                launch_ros.substitutions.FindPackageShare("tuw_gazebo_models"),
                                "launch",
                                "spawn_robot.launch.py"
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": "",
                    "robot": "pioneer3dx",
                    "X": "6.0",
                    "Y": "6.0",
                    "Theta": "3.1415",
                    "use_sim_time": "true",
                }.items(),
            ),
            launch_ros.actions.Node(
                name="slam_toolbox",
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                output="screen",
                parameters=[
                    os.path.join(config_dir, "slam.yaml"),
                    {"use_sim_time": True}
                ],
            ),
        ]
    )
