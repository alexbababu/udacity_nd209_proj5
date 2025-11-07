#!/bin/sh

# Umgebungsvariablen für TurtleBot setzen
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/alexander/workspace/project5/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
export ROBOT_INITIAL_POSE="-x 0 -y 0 -z 0"
CATKIN_WS_PATH="/home/alexander/workspace/project5/catkin_ws"
CUSTOM_WORLD="${CATKIN_WS_PATH}/src/world/world_project1.world"

# Basis-Setup, das in jedem Terminal ausgeführt wird.
# Wichtig: Wir nutzen 'bash -c' in xterm, um mehrere Befehle auszuführen.
ROS_SETUP="source /opt/ros/kinetic/setup.bash; source /home/alexander/workspace/project5/catkin_ws/devel/setup.bash"

# 1. ROS Master starten (roscore)
xterm -e "bash -c '$ROS_SETUP; roscore; exec bash'" & 
sleep 7

# 2. TurtleBot in der Welt starten (turtlebot_world.launch)
# Dies startet Gazebo. Wir geben ihm extra Zeit zum Laden.
xterm -e "bash -c '$ROS_SETUP; $ROBOT_INITIAL_POSE; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$CUSTOM_WORLD; exec bash'" & 
#xterm -e "bash -c '$ROS_SETUP; roslaunch turtlebot_gazebo turtlebot_world.launch; exec bash'" & 
sleep 15

# 3. SLAM-Knoten starten (gmapping_demo.launch)
xterm -e "bash -c '$ROS_SETUP; roslaunch turtlebot_gazebo gmapping_demo.launch; exec bash'" &
sleep 5

# 4. RViz zur Anzeige der Navigation starten (view_navigation.launch)
xterm -e "bash -c '$ROS_SETUP; roslaunch turtlebot_rviz_launchers view_navigation.launch; exec bash'" &
sleep 5

# 5. Teleoperation (Steuerung über die Tastatur) starten
# Dieses Terminal bleibt im Vordergrund für deine Eingaben.
xterm -e "bash -c '$ROS_SETUP; roslaunch turtlebot_teleop keyboard_teleop.launch; exec bash'"
