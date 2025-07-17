#!/bin/bash

#Open first terminal and run Localization
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; ros2 launch turtlebot4_navigation localization.launch.py map:=office.yaml; exec bash"

sleep 3

# Open second terminal and run Nav2
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; ros2 launch turtlebot4_navigation nav2.launch.py; exec bash"


# Open third terminal and run Robot GUI
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source ~/turtlebot4_ws/install/setup.bash; ros2 run turtlebot4_python robot_gui; exec bash"


# Open first terminal and run Autonomos Robot
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source ~/turtlebot4_ws/install/setup.bash; ros2 run turtlebot4_python move_robot; exec bash"
