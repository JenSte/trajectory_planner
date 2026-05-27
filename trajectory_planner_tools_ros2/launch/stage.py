# Launch only the planner, not the complete navigation stack.

import os.path

from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.launch_description_sources
import launch_ros.actions


def generate_launch_description():

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time',  default='true')


    package_dir = get_package_share_directory("trajectory_planner_tools")


    map_dir = os.path.join(package_dir, "map")
    config_dir = os.path.join(package_dir, "config")

    amcl_configuration = os.path.join(config_dir, "amcl.yaml")
    rviz_configuration =  os.path.join(config_dir, "stage.rviz")

    stage_ros_directory = get_package_share_directory('stage_ros2')
    world_file = os.path.join(stage_ros_directory, "world", "cave.world")

    config_file_argument = launch.actions.DeclareLaunchArgument(
        "config_file",
        description="The planner_server configuration file.",
        default_value="standalone.yaml",
    )

    map_name_argument = launch.actions.DeclareLaunchArgument(
        "map_name",
        description="The map to load.",
        default_value="cave_2cm.yaml",
    )

    config_file = launch.substitutions.LaunchConfiguration("config_file")
    map_name = launch.substitutions.LaunchConfiguration("map_name")

    return launch.LaunchDescription(
        [
            config_file_argument,
            map_name_argument,
            #launch.actions.SetEnvironmentVariable(
            #    "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
            #),
            #launch_ros.actions.Node(
            #    name="planner_server",
            #    package="nav2_planner",
            #    executable="planner_server",
            #    output="screen",
            #    parameters=[
            #        {
            #            "use_sim_time": use_sim_time,
            #        },
            #        launch.substitutions.PathJoinSubstitution(
            #            [config_dir, config_file]
            #        )
            #    ],
            #),
            #launch_ros.actions.Node(
            #    name="lifecycle_manager",
            #    package="nav2_lifecycle_manager",
            #    executable="lifecycle_manager",
            #    output="screen",
            #    parameters=[
            #        {
            #            "use_sim_time": use_sim_time,
            #            "autostart": True,
            #            #"node_names": ["map_server", "planner_server"],
            #            "node_names": ["map_server", "amcl"],
            #        }
            #    ]
            #),
            #launch_ros.actions.Node(
            #    name="static_transform_map_base_link",
            #    package="tf2_ros",
            #    executable="static_transform_publisher",
            #    output="screen",
            #    arguments=6 * ["0.0"] + ["map", "odom"],
            #    parameters=[
            #        {
            #            "use_sim_time": use_sim_time,
            #        }
            #    ]
            #),
            #launch_ros.actions.Node(
            #     name="amcl",
            #     package="nav2_amcl",
            #     executable="amcl",
            #     output="screen",
            #     parameters=[amcl_configuration],
            #),
            #launch_ros.actions.Node(
            #    name="map_server",
            #    package="nav2_map_server",
            #    executable="map_server",
            #    output="screen",
            #    # arguments=["--ros-args", "--log-level", "debug"],
            #    parameters=[
            #        {
            #            "use_sim_time": use_sim_time,
            #            "yaml_filename": launch.substitutions.PathJoinSubstitution(
            #                [map_dir, map_name]
            #            )
            #        },
            #    ],
            #),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name='rviz2',
                arguments=["-d", rviz_configuration],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="stage_ros2",
                executable="stage_ros2",
                name="stage",
                parameters=[
                    {
                        #'use_stamped_velocity': use_stamped_velocity,
                        #'use_ackermann': use_ackermann,
                        "enforce_prefixes": False,
                        #'use_static_transformations': use_static_transformations,
                        "one_tf_tree": False,
                        "world_file": world_file
                    }
                ],
            ),
        ]
    )
