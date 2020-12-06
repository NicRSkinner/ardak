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

ament_export_dependencies(rclcpp rclcpp_components bfr_base)
set(SOURCES
  src/input_calibration_node.cpp
  src/ardak_node.cpp
)

# add components
add_library(ardak_component SHARED
  src/ardak_node.cpp)

add_library(input_calibration_component SHARED
  src/input_calibration_node.cpp)
  
ament_target_dependencies(ardak_component
    "rclcpp"
    "rclcpp_components")
ament_target_dependencies(input_calibration_component
  "rclcpp"
  "rclcpp_components")

rclcpp_components_register_nodes(ardak_component "bfr::ArdakNode")
rclcpp_components_register_nodes(input_calibration_component "bfr::InputCalibrationNode")


include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(main src/main.cpp ${SOURCES})
ament_target_dependencies(main bfr_base rclcpp rclcpp_components std_msgs)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
