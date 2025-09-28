#!/bin/bash
gnome-terminal \
--tab -e "bash -c 'source $HOME/.bashrc;sudo systemctl stop start_app_node.service;ros2 launch slam slam.launch.py enable_save:=false'" \
--tab -e "bash -c 'source $HOME/.bashrc;sleep 10;ros2 launch peripherals teleop_key_control.launch.py'" \
--tab -e "bash -c 'source $HOME/.bashrc;sleep 10;rviz2 rviz2 -d /home/wentao/elec555_ws/src/slam/rviz/slam_desktop.rviz'" \
--tab -e "bash -c 'source $HOME/.bashrc;sleep 10;ros2 run slam map_save'"
