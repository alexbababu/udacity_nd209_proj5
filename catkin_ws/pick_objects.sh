#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/workspace/project5/catkin_ws/devel/setup.bash

MAP_PATH="$(rospack find my_nav_config)/maps/my_gazebo_map.yaml"
WORLD_PATH="$(rospack find my_nav_config)/world/world_project1.world"

export ROBOT_INITIAL_POSE="-x 0.0 -y 0.0 -z 0.0 -Y 0.0" 

xterm -e "bash -c ' roscore; exec bash'" & 
sleep 7

xterm -e "bash -c 'roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$WORLD_PATH; exec bash'" &
sleep 25 

xterm -e "bash -c 'roslaunch my_nav_config my_amcl_demo.launch map_file:=$MAP_PATH; exec bash'" &
sleep 5 

xterm -e "bash -c 'roslaunch turtlebot_rviz_launchers view_navigation.launch; exec bash'" &
sleep 5 

xterm -e "bash -c 'rosrun pick_objects pick_objects; exec bash'" 


