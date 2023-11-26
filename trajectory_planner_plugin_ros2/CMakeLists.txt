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
    "../trajectory_planner/include/trajectory_planner/turn_costs.hpp"
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
    "../trajectory_planner/src/trajectory_planner_debug_data.cpp"
    "../trajectory_planner/src/trajectory_planner_debug_images.cpp"
    "../trajectory_planner/src/turn_costs.cpp"
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

#if(BUILD_TESTING)
#    find_package(ament_lint_auto REQUIRED)
#    ament_lint_auto_find_test_dependencies()
#
#    find_package(ament_cmake_gmock REQUIRED)
#    ament_add_gmock(${PROJECT_NAME}_test
#        "../trajectory_planner/include/trajectory_planner/a_star.hpp"
#        "../trajectory_planner/include/trajectory_planner/buffer.hpp"
#        "../trajectory_planner/include/trajectory_planner/convolution.hpp"
#        "../trajectory_planner/include/trajectory_planner/costs.hpp"
#        "../trajectory_planner/include/trajectory_planner/heuristic.hpp"
#        "../trajectory_planner/include/trajectory_planner/motion_model.hpp"
#        "../trajectory_planner/include/trajectory_planner/planning_3d.hpp"
#        "../trajectory_planner/include/trajectory_planner/planning_5d.hpp"
#        "../trajectory_planner/include/trajectory_planner/trajectory_planner.hpp"
#        "../trajectory_planner/include/trajectory_planner/types.hpp"
#        "../trajectory_planner/src/buffer.cpp"
#        "../trajectory_planner/src/convolution.cpp"
#        "../trajectory_planner/src/costs.cpp"
#        "../trajectory_planner/src/heuristic.cpp"
#        "../trajectory_planner/src/motion_model.cpp"
#        "../trajectory_planner/src/planning_3d.cpp"
#        "../trajectory_planner/src/planning_5d.cpp"
#        "../trajectory_planner/src/trajectory_planner.cpp"
#        "../trajectory_planner/src/types.cpp"
#        "test/motion_model_fixtures.cpp"
#        "test/motion_model_fixtures.hpp"
#        "test/test_a_star.cpp"
#        "test/test_convolution.cpp"
#        "test/test_costs.cpp"
#        "test/test_motion_model.cpp"
#        "test/test_planning_3d.cpp"
#        "test/test_planning_5d.cpp"
#    )
#
#    target_include_directories(${PROJECT_NAME}_test
#        PRIVATE
#            "../trajectory_planner/include"
#            "include"
#            ${Boost_INCLUDE_DIR}
#            ${GMOCK_INCLUDE_DIRS}
#            ${OpenCV_INCLUDE_DIRS}
#            PkgConfig::FFTW
#    )
#
#    target_link_libraries(${PROJECT_NAME}_test
#        ${GMOCK_LDFLAGS}
#        ${OpenCV_LIBS}
#        Boost::iostreams
#        Boost::serialization
#        Boost::system
#        PkgConfig::FFTW
#    )
#endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
