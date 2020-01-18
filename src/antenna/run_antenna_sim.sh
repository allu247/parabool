#!/bin/sh

gnome-terminal --tab -e "sh -c 'printf \"\033]0;Antenna_description\007\";roslaunch antenna_description antenna_world.launch'" \
			   --tab -e "sh -c 'sleep 10; printf \"\033]0;Antenna_control\007\"; roslaunch antenna_control antenna_control.launch'" \
			   --tab -e "sh -c 'sleep 12; printf \"\033]0;Antenna_controller\007\"; rosrun antenna_controller antenna_controller_node.py'" \
			   --tab -e "sh -c 'sleep 12; printf \"\033]0;Satellite_chooser\007\"; rosrun antenna_controller satellite_chooser_node.py'" \
			   --tab -e "sh -c 'sleep 15; printf \"\033]0;Satellite_tracker\007\"; roslaunch antenna_controller antenna_controller.launch'"
