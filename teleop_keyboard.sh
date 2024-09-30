#!/bin/bash       

echo "teleop_twist_keyboard over topic /cmd_vel_teleop for mux"
echo
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_teleop