cmake_minimum_required(VERSION 3.9)
project(trajectory_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ftree-vectorize -march=native -mtune=native")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost 1.65 REQUIRED COMPONENTS iostreams serialization system)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_planner_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

pkg_search_module(FFTW REQUIRED fftw3 IMPORTED_TARGET)

set(DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav2_msgs
    nav2_util
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    rosidl_default_runtime
    std_msgs
    tf2
    tf2_ros
    trajectory_planner_msgs
    visualization_msgs
)

add_library(${PROJECT_NAME}_plugin SHARED
    "include/trajectory_planner/a_star.hpp"
    "include/trajectory_planner/buffer.hpp"
    "include/trajectory_planner/convolution.hpp"
    "include/trajectory_planner/costs.hpp"
    "include/trajectory_planner/heuristic.hpp"
    "include/trajectory_planner/motion_model.hpp"
    "include/trajectory_planner/planning_3d.hpp"
    "include/trajectory_planner/planning_5d.hpp"
    "include/trajectory_planner/trajectory_planner.hpp"
    "include/trajectory_planner/trajectory_planner_plugin.hpp"
    "include/trajectory_planner/types.hpp"
    "src/buffer.cpp"
    "src/convolution.cpp"
    "src/costs.cpp"
    "src/heuristic.cpp"
    "src/motion_model.cpp"
    "src/planning_3d.cpp"
    "src/planning_5d.cpp"
    "src/trajectory_planner.cpp"
    "src/trajectory_planner_plugin.cpp"
    "src/types.cpp"
)

target_include_directories(${PROJECT_NAME}_plugin
    PRIVATE
        "include"
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        PkgConfig::FFTW
)

target_link_libraries(${PROJECT_NAME}_plugin
    ${OpenCV_LIBS}
    Boost::iostreams
    Boost::serialization
    Boost::system
    PkgConfig::FFTW
)

target_compile_definitions(${PROJECT_NAME}_plugin
    PUBLIC
        "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

set_property(TARGET ${PROJECT_NAME}_plugin
    PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE
)

ament_target_dependencies(${PROJECT_NAME}_plugin
    ${DEPENDENCIES}
)

pluginlib_export_plugin_description_file(
    nav2_core ${PROJECT_NAME}_plugin.xml
)

install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${PROJECT_NAME}_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION include)

if(BUILD_TESTING)
    pkg_search_module(GMOCK REQUIRED gmock)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test
        "include/trajectory_planner/a_star.hpp"
        "include/trajectory_planner/buffer.hpp"
        "include/trajectory_planner/convolution.hpp"
        "include/trajectory_planner/costs.hpp"
        "include/trajectory_planner/heuristic.hpp"
        "include/trajectory_planner/motion_model.hpp"
        "include/trajectory_planner/planning_3d.hpp"
        "include/trajectory_planner/planning_5d.hpp"
        "include/trajectory_planner/trajectory_planner.hpp"
        "include/trajectory_planner/trajectory_planner_plugin.hpp"
        "include/trajectory_planner/types.hpp"
        "src/buffer.cpp"
        "src/convolution.cpp"
        "src/costs.cpp"
        "src/heuristic.cpp"
        "src/motion_model.cpp"
        "src/planning_3d.cpp"
        "src/planning_5d.cpp"
        "src/trajectory_planner.cpp"
        "src/types.cpp"
        "test/motion_model_fixtures.cpp"
        "test/motion_model_fixtures.hpp"
        "test/test_a_star.cpp"
        "test/test_convolution.cpp"
        "test/test_costs.cpp"
        "test/test_motion_model.cpp"
        "test/test_planning_3d.cpp"
        "test/test_planning_5d.cpp"
    )

    target_include_directories(${PROJECT_NAME}_test
        PRIVATE
            "include"
            ${Boost_INCLUDE_DIR}
            ${GMOCK_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            PkgConfig::FFTW
    )

    target_link_libraries(${PROJECT_NAME}_test
        ${GMOCK_LDFLAGS}
        ${OpenCV_LIBS}
        Boost::iostreams
        Boost::serialization
        Boost::system
        PkgConfig::FFTW
    )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
