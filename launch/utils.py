# Launch utility nodes to aid in the development in the trajectory planner.

import launch
import launch.actions
import launch_ros.actions


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch.actions.SetEnvironmentVariable(
                "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
            ),
            launch_ros.actions.Node(
                name="animate_path",
                package="trajectory_planner",
                executable="animate_path.py",
                output="screen",
            ),
            launch_ros.actions.Node(
                name="plot_augmented_path",
                package="trajectory_planner",
                executable="plot_augmented_path.py",
                output="screen",
            ),
            launch.actions.ExecuteProcess(
                name="echo_clicked_point",
                cmd=["ros2", "topic", "echo", "--field", "point", "/clicked_point", "geometry_msgs/msg/PointStamped"],
                output="screen",
                additional_env={"PYTHONUNBUFFERED": "1"},
            ),
        ]
    )
