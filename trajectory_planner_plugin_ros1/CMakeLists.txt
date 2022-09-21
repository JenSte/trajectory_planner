cmake_minimum_required(VERSION 3.9)
project(trajectory_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        rosconsole
        roscpp
        tf2_ros
        sensor_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        navfn
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        rosconsole
        roscpp
        tf2_ros
        sensor_msgs
)

add_library(${PROJECT_NAME}_plugin SHARED
    "include/trajectory_planner_plugin/trajectory_planner_plugin.hpp"
    "src/trajectory_planner_plugin.cpp"
)

target_include_directories(${PROJECT_NAME}_plugin
    PRIVATE
        "include"
        "../trajectory_planner/include"
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_plugin
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES trajectory_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
