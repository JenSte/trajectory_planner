cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_tools)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY config launch map
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
