#!/bin/bash

# 运行 cd ..
cd "$(dirname "$0")/../map"

# 运行 ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=map.yaml
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=map.yaml &

# 新建一个终端，运行 rviz2
xterm -e "rviz2" &

# 新建一个终端，运行 ros2 lifecycle set /map_server configure 和 ros2 lifecycle set /map_server activate
xterm -e "ros2 lifecycle set /map_server configure; ros2 lifecycle set /map_server activate" &
