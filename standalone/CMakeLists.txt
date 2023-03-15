cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_standalone)

#
# Stand alone executable using the trajectory planner without any ROS dependencies.
#

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost 1.65 REQUIRED COMPONENTS iostreams serialization system)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PNG REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)

add_executable(${PROJECT_NAME}
    "../trajectory_planner/include/trajectory_planner/a_star.hpp"
    "../trajectory_planner/include/trajectory_planner/buffer.hpp"
    "../trajectory_planner/include/trajectory_planner/convolution.hpp"
    "../trajectory_planner/include/trajectory_planner/costs.hpp"
    "../trajectory_planner/include/trajectory_planner/heuristic.hpp"
    "../trajectory_planner/include/trajectory_planner/motion_model.hpp"
    "../trajectory_planner/include/trajectory_planner/planning_3d.hpp"
    "../trajectory_planner/include/trajectory_planner/planning_5d.hpp"
    "../trajectory_planner/include/trajectory_planner/trajectory_planner.hpp"
    "../trajectory_planner/include/trajectory_planner/types.hpp"
    "../trajectory_planner/src/buffer.cpp"
    "../trajectory_planner/src/convolution.cpp"
    "../trajectory_planner/src/costs.cpp"
    "../trajectory_planner/src/heuristic.cpp"
    "../trajectory_planner/src/motion_model.cpp"
    "../trajectory_planner/src/planning_3d.cpp"
    "../trajectory_planner/src/planning_5d.cpp"
    "../trajectory_planner/src/trajectory_planner.cpp"
    "../trajectory_planner/src/trajectory_planner_debug_images.cpp"
    "../trajectory_planner/src/types.cpp"
    "standalone.cpp"
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        "../trajectory_planner/include"
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        PNG::PNG
        PkgConfig::FFTW
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Boost::iostreams
    Boost::serialization
    Boost::system
    PNG::PNG
    PkgConfig::FFTW
    Threads::Threads
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        "-g"
        "-march=native"
        "-mtune=native"
)

#set_property(TARGET ${PROJECT_NAME}
#    PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE
#)
