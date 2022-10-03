cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
)

add_message_files(
    DIRECTORY
        msg
    FILES
        "AugmentedPose.msg"
        "AugmentedPath.msg"
        "MotionModel.msg"
)

generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)
