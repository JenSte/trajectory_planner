cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_standalone)

#
# Stand alone executable using the trajectory planner without any ROS dependencies.
#

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost 1.65 REQUIRED COMPONENTS iostreams serialization system)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)

add_executable(${PROJECT_NAME}
    "../include/trajectory_planner/a_star.hpp"
    "../include/trajectory_planner/buffer.hpp"
    "../include/trajectory_planner/convolution.hpp"
    "../include/trajectory_planner/costs.hpp"
    "../include/trajectory_planner/heuristic.hpp"
    "../include/trajectory_planner/motion_model.hpp"
    "../include/trajectory_planner/planning_3d.hpp"
    "../include/trajectory_planner/planning_5d.hpp"
    "../include/trajectory_planner/trajectory_planner.hpp"
    "../include/trajectory_planner/types.hpp"
    "../src/buffer.cpp"
    "../src/convolution.cpp"
    "../src/costs.cpp"
    "../src/heuristic.cpp"
    "../src/motion_model.cpp"
    "../src/planning_3d.cpp"
    "../src/planning_5d.cpp"
    "../src/trajectory_planner.cpp"
    "../src/types.cpp"
    "standalone.cpp"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        "../include"
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        PkgConfig::FFTW
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Boost::iostreams
    Boost::serialization
    Boost::system
    PkgConfig::FFTW
    Threads::Threads
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        "-g"
        "-march=native"
        "-mtune=native"
)

set_property(TARGET ${PROJECT_NAME}
    PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE
)
