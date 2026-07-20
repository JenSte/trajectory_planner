cmake_minimum_required(VERSION 3.9)
project(trajectory_planner_tools)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_planner_msgs REQUIRED)

install(PROGRAMS
    script/animate_path.py
    script/client.py
    script/forward_simulation.py
    script/ground_truth_localization.py
    script/plot_augmented_path.py
    script/utils.py
    script/publish-goal-a.sh
    script/publish-goal-b.sh
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY world  DESTINATION share/${PROJECT_NAME})

ament_package()
