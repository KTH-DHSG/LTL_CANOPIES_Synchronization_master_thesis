# ltl_coordination

## Overview
This metapackage provides an implementation of a LTL (Linear Temporal Logic) planner based on automaton graph. More information can be found in each package documentation. This package represent a ROS2 update of the [ltl_automaton_core](https://github.com/KTH-DHSG/ltl_automaton_core) package for ROS2 with added functionalities for multi-robot coordination and synchronization.

### Publication

D. Peron, V. Nan Ferdandez-Ayala, E. V. Vlahakis, and D. V. Dimarogonas, "Efficient Coordination and Synchronization of Multi-Robot Systems Under Recurring Linear Temporal Logic", **TODO: ADDING DETAILS OF THE PUBLICATION ONCE AVAILABLE**


The original ROS planner is detailed in the following publication:

R. Baran, X. Tan, P. Varnai, P. Yu, S. Ahlberg, M. Guo, W. Shaw Cortez, and D. V. Dimarogonas, "A ROS Package for Human-In-the-Loop Planning and Control under Linear Temporal Logic Tasks", presented at the IEEE 17th International Conference on Automation Science and Engineering (CASE), 2021. To cite this work:
```
@INPROCEEDINGS{9551648,
  author={Baran, Robin and Tan, Xiao and Varnai, Peter and Yu, Pian and Ahlberg, Sofie and Guo, Meng and Cortez, Wenceslao Shaw and Dimarogonas, Dimos V.},
  booktitle={2021 IEEE 17th International Conference on Automation Science and Engineering (CASE)}, 
  title={A ROS Package for Human-In-the-Loop Planning and Control under Linear Temporal Logic Tasks}, 
  year={2021},
  volume={},
  number={},
  pages={2182-2187},
  doi={10.1109/CASE49439.2021.9551648}}
```



## Installation

### Dependencies
In the following we will specify specifi version for the software used to guarantee compatibility among them
- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/humble/index.html), package tested on Humble distribution on Ubuntu 22.04

- [LTL2BA](https://github.com/KTH-DHSG/ros_ltl2ba). ROS2 package wrapping for the LTL2BA software by Dennis Oddoux and Paul Gastin.
    - Clone the repository from Github in your ros workspace:
    ```
    cd ros2_ws/src
    git clone --branch ros2 https://github.com/KTH-DHSG/ros_ltl2ba.git
    ```

- [PLY (Python Lex-Yacc)](http://www.dabeaz.com/ply/) (Version 3.11 Required)
	```
  pip3 install ply==3.11
  ```

- [NetworkX](https://networkx.org/). Software for complex networks analysis in Python (Version 2.4 Required).
	```
  pip3 install networkx==2.4
  ````

- [PyYAML](https://pyyaml.org/). (Version 5.4.1 Required).
	```
  pip3 install pyyaml==5.4.1
  ```

### Specific Dependencies for ltl_automaton_synchronization and synchronization_experiments
- [CasADi](https://web.casadi.org/). Open-source tool for nonlinear optimization and algorithmic differentiation (Version 3.6.5 Required).
  ```
  pip3 install casadi==3.6.5
  ```

- [gurobipy](https://pypi.org/project/gurobipy/). Optimization solver (Version 11.0.1 Required).
  ```
  pip3 install gurobipy==11.0.1
  ```

- [SciPy](https://scipy.org/). Software used in the manipulation (Version 1.8.0 Required).
  ```
  pip3 install scipy==1.8.0
  ```

- [NumPy](https://numpy.org/). (Version 1.21.5 Required).
  ```
  pip3 install numpy=1.21.5
  ```

- [Matplotlib] (https://matplotlib.org/). Software used to plot the results of experiments. (Version 3.5.1 Riquired).
  ```
  pip3 install matplotlib=3.5.1
  ```

- [CycloneDDS](https://cyclonedds.io/) implementation for ROS2. Required for simulation with a high number of robots or experiments in the lab. It removes big comunication delays in the experimental setup and the use or more agents with reliable comunication in the simulations.
  ```
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  ```
  and then export in each terminal window the user will start a node with:
  ```
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
  or equivalently add it to `~/.bashrc` to run the command on evry new window opend with:
  ```
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
  ```

- [ros2_domain_bridge](https://github.com/ros2/domain_bridge/tree/humble?tab=readme-ov-file) Bridge to allow the comunication between different `ROS_DOMAIN_ID` this is required in the manipulation since the code developed for that tas was not compatible with CycloneDDS and to avoid the delays it was necessary to use a different `ROS_DOMAIN_ID` for the robot that executes the manipulation. 
```
sudo apt install ros-humble-domain-bridge
```
The configuration file for our specific scenario is avilable [here](/synchronization_experiments/synchronization_experiments/launch/paper/bridge_config.yaml). To run the bridge by itself use:
```
ros2 run domain_bridge domain_bridge ~/ros2_ws/src/LTL_CANOPIES_Synchronization_master_thesis/synchronization_experiments/synchronization_experiments/launch/paper/bridge_config.yaml
```
  
### Building
To build the package, clone the current repository in your ros2 workspace and build it.
```
cd ros2_ws/src
git clone https://github.com/KTH-DHSG/LTL_CANOPIES_Synchronization_master_thesis.git
```
Build your workspace with *colcon build*
```
cd ...
colcon build
```

## Usage

This package is not meant to be used on its own. It provides an LTL planner node that can be interfaced with an external agent package. Nevertheless an example launch file can be run using the following command:

```
ros2 launch ltl_automaton_planner ltl_planner_example_launch.py

```

It will launch a planner using the formula provided in the launch file *ltl_automaton_planner/launch/ltl_planner_example_launch.py* and using the transition system specified in *ltl_automaton_planner/config/example_ts*.

### ltl_automaton_synchronization usage

For the details on how to use this specific package refere to the [synchronization_experiments](/synchronization_experiments/) package.

## Packages
This metapackage is composed of the following packages.

- **[ltl_automaton_planner](/ltl_automaton_planner)**: Provides the LTL planner node. The node uses a transition system and a LTL formula to generate a plan and action sequence, and update them according to agent state.

- **[ltl_automaton_msg_srv](/ltl_automaton_msg_srv)**: A message definition packages for the LTL automaton packages.

- **[ltl_automaton_std_transition_systems](/ltl_automaton_std_transition_systems)**: A set of state monitors for standard transition systems (2D regions, ...).

- **[ltl_automaton_hil_mic](/ltl_automaton_hil_mic)**: An implementation of a Human-In-the-Loop mix initiative controller for agents using the LTL planner.

- **[ltl_automaton_synchronization](\ltl_automaton_synchronization)**: An implementation of the cooridnation and synchronization approach devloped.

- **[synchronization_experiments](\synchronization_experiments)**: A package including experimetns and simulations code used in the publication.

-**[rosie_pick_and_place_interfaces](\rosie_pick_and_place_interfaces)**: An interfaces package to enable pick and place operations with the Hebi Rosies available in the lab.