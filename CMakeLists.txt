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

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)


set(node_plugins "")

# add components
add_library(ardak_component SHARED
  src/ardak_node.cpp)
target_compile_definitions(ardak_component
  PRIVATE "ARDAK_DLL")
ament_target_dependencies(ardak_component
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(ardak_component "bfr::ArdakNode")
set(node_plugins "${node_plugins}bfr::ArdakNode;$<TARGET_FILE:ardak_component>\n")

add_library(input_calibration_component SHARED
  src/input_calibration_node.cpp)
target_compile_definitions(input_calibration_component
  PRIVATE "ARDAK_DLL")
ament_target_dependencies(input_calibration_component
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(input_calibration_component "bfr::InputCalibrationNode")
set(node_plugins "${node_plugins}bfr::InputCalibrationNode;$<TARGET_FILE:input_calibration_component>\n")

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(main src/main.cpp)

target_link_libraries(main
  ardak_component
  input_calibration_component)

ament_target_dependencies(main rclcpp)

install(TARGETS
  ardak_component
  input_calibration_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
