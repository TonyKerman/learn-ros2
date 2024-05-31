cmake_minimum_required(VERSION 3.8)
project(r2v2_behavior_tree)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rc2024_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(library_name ${PROJECT_NAME})

include_directories(
  include/r2v2_behavior_tree
)
set(dependencies
  rclcpp
  rclcpp_action
#   rclcpp_lifecycle
#   geometry_msgs
#   sensor_msgs
  nav2_msgs
#   nav_msgs
  behaviortree_cpp_v3
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   std_msgs
#   std_srvs
  nav2_util
  nav2_behavior_tree
  rc2024_interfaces
)
# add_library(mynav2_wait_action_bt_node SHARED src/mywait_action.cpp)
# list(APPEND plugin_libs mynav2_wait_action_bt_node)
# add_library(r2v2_putball_action_bt_node SHARED src/putball_action.cpp)
# list(APPEND plugin_libs r2v2_putball_action_bt_node)
# add_library(r2v2_catchball_action_bt_node SHARED src/catchball_action.cpp)
# list(APPEND plugin_libs r2v2_catchball_action_bt_node)
add_library(r2v2_spin_to_action_bt_node SHARED src/spin_to_action.cpp)
list(APPEND plugin_libs r2v2_spin_to_action_bt_node)
add_library(r2v2_revolve_joints_action_bt_node SHARED src/revolve_joints_action.cpp)
list(APPEND plugin_libs r2v2_revolve_joints_action_bt_node)
foreach(bt_plugin ${plugin_libs})
    ament_target_dependencies(${bt_plugin} ${dependencies})
    target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})



ament_package()
