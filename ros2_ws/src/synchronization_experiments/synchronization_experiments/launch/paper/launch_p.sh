#!/bin/bash

# Define the ROS 2 launch commands you want to run
declare -a commands=(
    "ros2 launch synchronization_experiments  data_collector_p.py"
    "ros2 launch synchronization_experiments  rosie_0_p.py"
    "ros2 launch synchronization_experiments  rosie_2_p.py"
    "ros2 launch synchronization_experiments  turtlebot_1_p.py"
    "ros2 launch synchronization_experiments  turtlebot_2_p.py"
    "ros2 launch synchronization_experiments  turtlebot_3_p.py"
    "ros2 launch synchronization_experiments  turtlebot_4_p.py"
    "ros2 launch synchronization_experiments  rosie_1_p.py"
    "ros2 launch synchronization_experiments  turtlebot_5_p.py"
    "ros2 launch synchronization_experiments  turtlebot_6_p.py"
    #"ros2 launch synchronization_experiments  turtlebot_7_p.py"
)
# Loop through the commands and open each in a new terminal window
for cmd in "${commands[@]}"
do
    xterm -hold -e "$cmd" &
    sleep 0.2
done
