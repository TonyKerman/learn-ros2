cmake_minimum_required(VERSION 3.5)
project(r2v2_behaviors)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(rc2024_interfaces REQUIRED)

include_directories(
  include
)

# set(library_name ${PROJECT_NAME}_plugin)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_core
  pluginlib
  nav2_behaviors
  CURL
  rc2024_interfaces
)

add_library(r2v2_spin_to_behavior SHARED
  src/spin_to.cpp
)
ament_target_dependencies(r2v2_spin_to_behavior
  ${dependencies}
)
add_library(r2v2_revolve_joints_behavior SHARED
  src/revolve_joints.cpp
)
ament_target_dependencies(r2v2_revolve_joints_behavior
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS 
                r2v2_spin_to_behavior
                r2v2_revolve_joints_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(r2v2_spin_to_behavior)
ament_export_libraries(r2v2_revolve_joints_behavior)
ament_export_dependencies(${dependencies})
ament_package()
