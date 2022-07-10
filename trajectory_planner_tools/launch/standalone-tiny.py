# Launch only the planner, not the complete navigation stack, in a configuration
# that uses a tiny robot footprint and a tiny map.

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.substitutions


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    [
                        launch.substitutions.PathJoinSubstitution(
                            [
                                launch_ros.substitutions.FindPackageShare("trajectory_planner"),
                                "launch",
                                "standalone.py"
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "config_file": "standalone-tiny.yaml",
                    "map_name": "tiny.yaml",
                }.items(),
            )
        ]
    )
