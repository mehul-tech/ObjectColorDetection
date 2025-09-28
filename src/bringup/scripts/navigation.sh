#!/bin/bash
gnome-terminal \
--tab -e "bash -c 'source $HOME/.bashrc;sudo systemctl stop start_app_node.service;rviz2 rviz2 -d /home/wentao/elec555_ws/src/navigation/rviz/navigation_desktop.rviz'" \
--tab -e "bash -c 'source $HOME/.bashrc;ros2 launch navigation navigation.launch.py map:=map_01'"
