cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_msgs)

unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AugmentedPose.msg"
    "msg/AugmentedPath.msg"
    "msg/MotionModel.msg"
    DEPENDENCIES
        geometry_msgs
)

ament_package()
