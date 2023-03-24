cmake_minimum_required(VERSION 3.9)
project(trajectory_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ftree-vectorize -march=native -mtune=native")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost 1.65 REQUIRED
    COMPONENTS
        iostreams
        serialization
        system
)

find_package(OpenCV REQUIRED
    COMPONENTS
        core
        imgcodecs
        imgproc
)

find_package(PkgConfig REQUIRED)
pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)

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
        trajectory_planner_msgs
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
        trajectory_planner_msgs
)

add_library(${PROJECT_NAME}_plugin SHARED
    "../trajectory_planner/include/trajectory_planner/a_star.hpp"
    "../trajectory_planner/include/trajectory_planner/buffer.hpp"
    "../trajectory_planner/include/trajectory_planner/convolution.hpp"
    "../trajectory_planner/include/trajectory_planner/costs.hpp"
    "../trajectory_planner/include/trajectory_planner/depth_heuristic.hpp"
    "../trajectory_planner/include/trajectory_planner/motion_model.hpp"
    "../trajectory_planner/include/trajectory_planner/planning_3d.hpp"
    "../trajectory_planner/include/trajectory_planner/planning_5d.hpp"
    "../trajectory_planner/include/trajectory_planner/planning_5d_heuristic.hpp"
    "../trajectory_planner/include/trajectory_planner/trajectory_planner.hpp"
    "../trajectory_planner/include/trajectory_planner/types.hpp"
    "../trajectory_planner/src/buffer.cpp"
    "../trajectory_planner/src/convolution.cpp"
    "../trajectory_planner/src/costs.cpp"
    "../trajectory_planner/src/depth_heuristic.cpp"
    "../trajectory_planner/src/motion_model.cpp"
    "../trajectory_planner/src/planning_3d.cpp"
    "../trajectory_planner/src/planning_5d.cpp"
    "../trajectory_planner/src/planning_5d_heuristic.cpp"
    "../trajectory_planner/src/trajectory_planner.cpp"
    "../trajectory_planner/src/trajectory_planner_costs.cpp"
    "../trajectory_planner/src/trajectory_planner_debug_images.cpp"
    "../trajectory_planner/src/types.cpp"
    "../trajectory_planner_plugin/include/trajectory_planner_plugin/convert.hpp"
    "../trajectory_planner_plugin/include/trajectory_planner_plugin/costmap_observer.hpp"
    "../trajectory_planner_plugin/include/trajectory_planner_plugin/plugin.hpp"
    "../trajectory_planner_plugin/src/convert.cpp"
    "../trajectory_planner_plugin/src/costmap_observer.cpp"
    "../trajectory_planner_plugin/src/plugin.cpp"
    "include/trajectory_planner_plugin/trajectory_planner_plugin.hpp"
    "src/trajectory_planner_plugin.cpp"
)

target_include_directories(${PROJECT_NAME}_plugin
    PRIVATE
        "../trajectory_planner/include"
        "../trajectory_planner_plugin/include"
        "include"
)
target_include_directories(${PROJECT_NAME}_plugin
    SYSTEM PRIVATE
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        PkgConfig::FFTW
)

target_link_libraries(${PROJECT_NAME}_plugin
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    Boost::iostreams
    Boost::serialization
    Boost::system
    PkgConfig::FFTW
)

# ROS1 had version macros in "ros/common.h", but this file
# does not exist in ROS2, so we have to use our own macro.
target_compile_definitions(${PROJECT_NAME}_plugin
    PUBLIC
        "ROS1"
)

set_property(TARGET ${PROJECT_NAME}_plugin
    PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE
)

install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES trajectory_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(${PROJECT_NAME}_node
    src/trajectory_planner_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
    PRIVATE
        "../trajectory_planner/include"
        "../trajectory_planner_plugin/include"
        "include"
)
target_include_directories(${PROJECT_NAME}_node
    SYSTEM PRIVATE
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        PkgConfig::FFTW
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_plugin
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
