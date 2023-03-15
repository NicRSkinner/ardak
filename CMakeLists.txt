cmake_minimum_required(VERSION 3.5)
project(ardak)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(bfr_base REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bfr_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(zyg_ai REQUIRED)

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_library(joint_state_source STATIC
  src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_source
  rclcpp tf2_ros std_msgs sensor_msgs
)

add_executable(main src/main.cpp src/ardak_node.cpp src/drive_controller_node.cpp)
ament_target_dependencies(main
  rclcpp
  bfr_base
  bfr_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2)

add_executable(robot_states src/robot_states.cpp)

target_link_libraries(
  robot_states
  joint_state_source
)

ament_target_dependencies(robot_states rclcpp robot_state_publisher)

install(TARGETS
  main
  robot_states
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src launch rviz config world description
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
