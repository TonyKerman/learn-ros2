#!/bin/bash

MICROROS_DIR="/home/wtr2023/ros_ws/microros_ws"
R2V2_DIR="/home/wtr2023/ros_ws/rc2024/RC24_R2V2"
CMD1="source install/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 1152000"
CMD="source install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py"

gnome-terminal -- bash -c "cd $MICROROS_DIR && $CMD1; exec bash"
gnome-terminal -- bash -c "cd $R2V2_DIR && $CMD; exec bash"



