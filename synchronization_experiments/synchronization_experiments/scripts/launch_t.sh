#!/bin/bash

# Define the ROS 2 launch commands you want to run
declare -a commands=(
    "ros2 launch synchronization_experiments  data_collector_t.py"
    "ros2 launch synchronization_experiments  rosie_0_t.py"
    "ros2 launch synchronization_experiments  rosie_2_t.py"
    "ros2 launch synchronization_experiments  rosie_1_t.py"
    "ros2 launch synchronization_experiments  turtlebot_1_t.py"
    "ros2 launch synchronization_experiments  turtlebot_2_t.py"


)
# Loop through the commands and open each in a new terminal window
for cmd in "${commands[@]}"
do
    xterm -hold -e "$cmd" &
    sleep 0.2
done
