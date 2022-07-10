cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_tools)

#unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)

find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED)
#find_package(rclpy REQUIRED)
#find_package(trajectory_planner_msgs REQUIRED)

install(PROGRAMS
    trajectory_planner_tools/animate_path.py
    trajectory_planner_tools/client.py
    trajectory_planner_tools/forward_simulation.py
    trajectory_planner_tools/plot_augmented_path.py
    trajectory_planner_tools/utils.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY map DESTINATION share/${PROJECT_NAME})

#ament_python_install_package(${PROJECT_NAME})

#ament_export_dependencies(${DEPENDENCIES})
ament_package()
