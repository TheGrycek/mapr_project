# MAPR_elevation_map_project

# Dependencies:

$ sudo apt-get install ros-melodic-map-server ros-melodic-dwa-local-planner libompl-dev

# Installation

$ cd ~/catkin_ws/src

$ git clone https://github.com/Kamilkim/MAPR_elevation_map_project.git

$ cd ~/catkin_ws/

$ catkin_make_isolated

# Run example

$ roslaunch MAPR_elevation_map_project MAPR_elevation_map_project.launch

# Tasks

1. run elevaiotn map

2. in the new terminal run script 

    $ rosrun MAPR_elevation_map_project elevMapEmple.py

3. add topic "reflected_map" in rviz

![Mapa](https://github.com/Kamilkim/MAPR_elevation_map_project/blob/master/doc/Elevation_map.JPG)
