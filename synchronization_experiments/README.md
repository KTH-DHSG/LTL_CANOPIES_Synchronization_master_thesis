# sunchronization_experiments
This package contains all the examples and experiments developed to work with the [ltl_automaton_synchronization](/ltl_automaton_synchronization/) package. It includes all the necessary launch files and scripts to perform the experiments and simulations described in the paper.

In the following when we refer to `$agent` this value must be substituted with the name of an agent.

## Config files
### Paper
Config files specific for the workspace described in the paper.
- `$agent.py` Generated motion and action dictionaries for the specific agents presented in the paper are  stored here. 

- `obstacles.py` Generated obstacle presented in the paper are stored here dictionary is stored here. 
### Thesis
Launch files specific for the workspace described in the thesis.
- `$agent.py` Generated motion and action dictionaries for the specific agents presented in the thesis are stored here. 

- `obstacles.py` Generated obstacle presented in the theis are stored here dictionary is stored here. 

## Launch files

### Paper
Launch files specific for the workspace described in the paper.

- `$agent_p.py` launch file for each robot during the the experiment. No arguments required to run.

- `generic_agent.py` launch file for a generic agent, used for the simulation with 90 agents. Required arguments:
    - `agents` (array of strings) representing the agents in the workspace.

    - `dictionary_file` (string) path of the file containing the dictionary.

    - `task` (string) a Recurrring LTL task assigned to the agent

    - `ns` (string) namespace of the agent equivalently the name of the agent

- `data_collector_p.py` launch file for the data collector node. No parameters required.

- `data_collector_p_multiple.py` launch file for the data collector node used for the simulation with 90 agnets. No parameters required. Required Arguments:
    - `agents` (array of strings) representing the agents in the workspace.

    - `agents_type` (array of strings) representing the type of the agents in the workspace, it must be in the same order as `agents`.

- `complexity_rrc.py` launch file for the Request Reply Confirmation cycle.

### Thesis
Launch files specific for the workspace described in the thesis.

- `$agent_t.py` launch file for each robot during the the experiment. No arguments required to run.

- `data_collector_t.py` launch file for the data collector node. No parameters required.

## Nodes
### data_collector.py
This node is used to save the actions executed by the agents during an experiment/simulations, the start of the collaborations and then every 10 seconds saving it into a `.pkl` file.

#### Subscribed Topics

- `$agent/next_move_cmd` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    The next move from the output word (action sequence) to be executed by the agent in order to fulfill the plan. Subscribes to the topic for each `$agent`.

- `$agent/synchro_start` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    Start topics where `agent` corresponds to the namespaces of agents in the workspace. These topics send a "start" message to the agents involved in a collaboration, allowing them to begin their actions.

#### Parameters
- `save_location` (string, default: ['data_collector.pkl'])

    Name of the file where we want to save the recorded data.

- `agents` (array of strings, default: [''])

    Set of agents in the workspace.

- `possible_agent_types` (array of strings, default: [''])
    
    Set containing all the possible types of agents.

- `agents_type` (array of strings, default: [''])
    
    Representing the type of the agents in the workspace, it must be in the same order as `agents`. All the elements must be of one of the types defined by `possible_agent_types`.

- `motion_action_dictionary_paths` (array of strings, default: [''])

    Array of path to the agent model transition system definitions. The files must be generated according to the action and motion models defined in the paper. it masu contain the same number of entries and in the same order as `possible_agent_types`.

### rrc_sim.py

Node used to test the performace of the request reply confirmation cycle and the effectiveness of the MIP filtering procedure.

#### Subscribed Topics
- `synchro_request` ([ltl_automaton_msg_srv/msg/SynchroRequest](/ltl_automaton_msg_srv/msg/SynchroRequest.msg))

    Request topic where each agent can publish or receive a message requesting help from other agents to complete an action. A request consists of a set of actions with respective regions where they must be completed and the time the action is supposed to start.

- `agent0/snchro_reply` ([ltl_automaton_msg_srv/msg/SynchroReply](/ltl_automaton_msg_srv/msg/SynchroReply.msg))

    Reply topic for `agent0`. This topic is used by `agent0` to recieve the replies from other agents.

#### Published Topics
- `synchro_request` ([ltl_automaton_msg_srv/msg/SynchroRequest](/ltl_automaton_msg_srv/msg/SynchroRequest.msg))

    Request topic where each agent can publish or receive a message requesting help from other agents to complete an action. A request consists of a set of actions with respective regions where they must be completed and the time the action is supposed to start.

- `agent0/snchro_reply` ([ltl_automaton_msg_srv/msg/SynchroReply](/ltl_automaton_msg_srv/msg/SynchroReply.msg))

    Reply topic for `agent0`. This topic is used by the agents to send their reply to `agent0`.

#### Parameters

- `num_agents` (double, default: 1)

    This parameter specifies how many agents will be involved in the simulation.

- `num_ass_actions` (double, default: 1)

    This parameter specifies how many assitive actions will be requested.

- `filter` (Bool, default: True)

    Flag used to execute the filtering procedure. If True, the filtering procedure will be executed otherwise it will not.

- `waiting` (double, default: 10)

    This parameter specifies the waiting time before sendig the request, it is used to guarantee that all agents are initialized. The highere the number of agents the higher the value of `waiting` will be necessary.

## Scripts

- `launch_p.sh` Bash script to run the paper experimental scenario. It icludes the commands for the `ros2_domain_bridge`, `data_collector_p.py` launch file, the agents' launch files and the `manipulation_node`.

- `launch_t.sh` Bash script to run the thesis experimental scenario. It icludes the commands for the `data_collector_t.py` launch file and the agents' launch files.

- `multi_agent_launch.py` Python script used to generate a file containing all the commands to run a simulation with multiple batches of 9 agents each as the one considered in the paper experiment. The globa variable `BATCHES` define the number of batches wile in the code it is possible to change the save location of the `.yaml` file where the commands will be saved. the commands are based on the `generic_agent.py` launch file.

- `launch_multiple.sh` Auxiliary bash script used to launch the scalability simulation. The commands generated with `multi_agent_launch.py` will need to be pasted in `commands` at the beginning of the script.

- `launch_launch.sh` Bash script used to launch the scalability simulation. It launches on different terminals `launch_multiple.sh` creating a terminal for each batch. It is responible also for launching the `data_collector_p_multiple.py` launch file.

- `convert_pkl_csv.py` Python script used to convert from/to `.pkl` to/from `.csv` the data saved by a `data_collector.py` node. It is used to have a different representation of the data that does not require to be plotted to be visible.

## Results
This folder contains the results of all the simulations and experiments conducted. It also contains the scripts used to obtain the plots presented in both thesis and paper.

## Recurring LTL Forumals Specification

This code will work with any type of LTL formula but to keep compliance of the LTL task assigned it is necessary to work with a Recurring LTL formula as the one presented in the theoretical work; Namely considering the following grammar:
$$\varphi' ::=\top \mid a \mid \neg a\mid \varphi'_1\wedge\varphi'_2 \mid \lozenge\varphi'.$$
A Recurring LTL formula is defined as:
$$\varphi_r=\varphi'_1\wedge\square\lozenge\varphi'_2.$$