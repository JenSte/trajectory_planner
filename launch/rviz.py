import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch_ros.actions

def generate_launch_description():

    package_dir = get_package_share_directory("trajectory_planner")
    config_dir = os.path.join(package_dir, "config")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                name="rviz2",
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=['-d', os.path.join(config_dir, "standalone.rviz")],
            ),
        ]
    )
