cmake_minimum_required(VERSION 3.8)
project(task_pkg)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uclv_dynamixel_utils REQUIRED)
find_package(uclv_seed_robotics_ros_interfaces REQUIRED)
find_package(repo_controller REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)


add_executable(task_node src/task_node.cpp)
ament_target_dependencies(
  task_node
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces" # std_msgs sta quì
  "std_srvs"
  "uclv_dynamixel_utils"
  "repo_controller"
)


add_executable(task_node2 src/task_node2.cpp)
ament_target_dependencies(
  task_node2
  "rclcpp"
  "uclv_seed_robotics_ros_interfaces" # std_msgs sta quì
  "std_srvs"
  "uclv_dynamixel_utils"
  "repo_controller"
  "rclcpp_action"
  "control_msgs"
  "trajectory_msgs"
)





install(TARGETS 
  task_node
  task_node2
  DESTINATION 
    lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION
    share/${PROJECT_NAME}/
)
  
install(DIRECTORY
  config
  DESTINATION 
    share/${PROJECT_NAME}
)
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
