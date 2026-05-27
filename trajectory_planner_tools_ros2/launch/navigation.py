import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        description="Whether or not to use simulation time.",
        default_value='true',
    )

    map_name_argument = DeclareLaunchArgument(
        "map_name",
        description="The map to load.",
        default_value="cave_2cm.yaml",
    )

    planner_configuration_argument = DeclareLaunchArgument(
        "planner_configuration",
        description="The configuration file for the planner server.",
        default_value="planner-trajectory-planner.yaml",
    )

    package_dir = get_package_share_directory("trajectory_planner_tools")
    config_dir = os.path.join(package_dir, "config")
    map_dir = os.path.join(package_dir, "map")

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': PathJoinSubstitution(
                    [
                        map_dir,
                        LaunchConfiguration("map_name"),
                    ],
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(config_dir, "amcl.yaml"),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    config_dir,
                    LaunchConfiguration("planner_configuration"),
                ]
            ),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            os.path.join(config_dir, "controller.yaml"),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            os.path.join(config_dir, "behaviors.yaml"),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(config_dir, "bt_navigator.yaml"),
            {
                'default_bt_xml_filename': os.path.join(config_dir, "behavior.xml"),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            os.path.join(config_dir, "waypoint_follower.yaml"),
        ],
    )

    livecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ]
            }
        ],
    )
    
    return LaunchDescription(
        [
            use_sim_time_argument,
            map_name_argument,
            planner_configuration_argument,
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            livecycle_manager_node,
        ]
    )
