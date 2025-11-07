# udacity_nd209_proj5: Home Service Robot

For this project, I set up a Virtual Machine running Ubuntu 16.04. I got a CATKIN workspace running and pulled down all the necessary TurtleBot packages from Git. These packages provided the full TurtleBot configuration (Lidar, depth camera) and the core ROS components needed for localization, mapping, and navigation.


## ROS Packages Used

This section outlines the key packages used to achieve the home service functionality:

- Simulation: turtlebot_gazebo This is the simulation environment. It provides the virtual world and sends the raw sensor data (like laser scans) that the localization and navigation packages use.

- Localization: AMCL This package handles the robot's localization. It uses a particle filter with the Lidar data to match what the robot "sees" against the static map, so it can determine its precise position and orientation (/pose) in the map frame.

- Navigation: move_base This is the main navigation stack. It manages global and local path planning. move_base uses the Dijkstra-Algorithmus in its Global Planner to find the most efficient path around static obstacles from start to finish.

- Configuration: my_nav_config I created this separate package to experiment with custom parameters for the AMCL setup. Although I ultimately commented most of them out because the 2D navigation worked fine with the defaults, it was necessary for organization.

## Custom C++ Logic (Mission Control)

Two custom C++ packages were developed to handle the mission flow:

- pick_objects (Mission Control): This C++ program functions as the Action Client. It sends the robot sequentially to the two main points on the map (pickup and drop-off). I made sure to add checks in the code to verify the goal coordinates were actually reachable on the map and not inside a wall.

- add_markers (Object Simulation): This package handles the virtual object simulation in Rviz.
	1. Initial Logic (Task 2): It first worked on a simple timer (show marker, wait 5 seconds, hide, wait 5 seconds, show at drop-off).
	2. Final Logic (Task 3): I then upgraded the program to listen to the robot's odometry data (/odom). This makes the marker actions dynamic: the marker now disappears exactly when the robot reaches the pickup spot and reappears at the drop-off spot once the robot reaches the second goal. This synchronization makes the simulation feel much more realistic.
