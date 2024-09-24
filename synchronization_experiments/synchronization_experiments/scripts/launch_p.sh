#!/bin/bash

# Define the ROS 2 launch commands you want to run
declare -a commands=(
    "ros2 run domain_bridge domain_bridge ~/LTL_CANOPIES_Synchronization_master_thesis/ros2_ws/src/synchronization_experiments/synchronization_experiments/launch/paper/bridge_config.yaml"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  data_collector_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  rosie_0_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  rosie_2_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  turtlebot_1_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  turtlebot_2_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  turtlebot_3_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  turtlebot_4_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  rosie_1_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  turtlebot_5_p.py"
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 launch synchronization_experiments  turtlebot_6_p.py"
    "export ROS_DOMAIN_ID=42; ros2 run ltl_automaton_synchronization manipulation_node"
    
)
# Loop through the commands and open each in a new terminal window
for cmd in "${commands[@]}"
do
    xterm -hold -e "$cmd" &
    sleep 0.2
done
