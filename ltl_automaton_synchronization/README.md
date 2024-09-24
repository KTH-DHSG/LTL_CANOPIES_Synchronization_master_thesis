# ltl_automaton_synchronization

## Overview
This package builds upon the [ltl_automaton_planner](/ltl_automaton_planner/) package by adding multi-agent coordination and synchronization capabilities. For all the basic functionalities and information, please refer to the [ltl_automaton_planner](/ltl_automaton_planner/).

Moreover, for the launch files and usage examples, please refer to the [synchronization_experiments](/synchronization_experiments) package.

In the following sections, the term `curr_agent` refers to the agent that started the node, while `agent` refers to any of the agents present in the workspace.

## Nodes
### synchro.py
This is an updated version of the planner node. In addition to the existing functions, it handles collaboration and synchronization of agents through a request-reply and confirmation mechanism. Below, we list only the topics that are added compared to the original planner node.

#### Subscribed Topics
- `synchro_request` ([ltl_automaton_msg_srv/msg/SynchroRequest](/ltl_automaton_msg_srv/msg/SynchroRequest.msg))

    Request topic where each agent can publish or receive a message requesting help from other agents to complete an action. A request is composed of a set of actions with respective regions where they must be completed and the time when the action is supposed to start.

- `curr_agent/reply` ([ltl_automaton_msg_srv/msg/SynchroReply](/ltl_automaton_msg_srv/msg/SynchroReply.msg))

    Reply topic of the current agent. Here, `curr_agent` refers to the namespace of the agent in the launch file that started the node.

- `synchro_confirm` ([ltl_automaton_msg_srv/msg/SynchroConfirm](/ltl_automaton_msg_srv/msg/SynchroConfirm.msg))

    Confirmation topic common to all agents in the workspace. It is used to send and receive confirmation messages that establish collaboration between agents.

- `next_move_cmd` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    The next move from the output word (action sequence) to be executed by the agent in order to fulfill the plan.

#### Published Topics
- `synchro_request` ([ltl_automaton_msg_srv/msg/SynchroRequest](/ltl_automaton_msg_srv/msg/SynchroRequest.msg))

    Request topic where each agent can publish or receive a message requesting help from other agents to complete an action. A request consists of a set of actions with respective regions where they must be completed and the time the action is supposed to start.

- `agent0/reply` ([ltl_automaton_msg_srv/msg/SynchroReply](/ltl_automaton_msg_srv/msg/SynchroReply.msg))

    Reply topics where `agent` corresponds to the namespaces of agents present in the workspace. These topics are used to send replies to agents requesting assistance.

- `synchro_confirm` ([ltl_automaton_msg_srv/msg/SynchroConfirm](/ltl_automaton_msg_srv/msg/SynchroConfirm.msg))

    A confirmation topic common to all agents in the workspace, used to send and receive confirmation messages establishing collaboration between agents.

#### Services
- `finished_collab` ([ltl_automaton_msg_srv/srv/FinishCollab](/ltl_automaton_msg_srv/srv/FinishCollab.srv))

    Service used to notify the planner when a collaborative action has been completed.

#### Parameters
- `motion_action_dictionary_path` (string)

    Path to the agent model transition system definition. The file must be generated according to the action and motion models defined in the paper. A script to generate the action and motion dictionary used in the paper can be found [here](/ltl_automaton_synchronization/ltl_automaton_synchronization/transition_systems/generate_dictionaries_paper.py).

- `agents` (array of strings, default: [''])

    Set of agents in the workspace.

- `time_horizon` (double, default: 10)

    This parameter specifies how many seconds ahead the planner should look for collaborative actions in the plan. It needs to be tuned according to the length of the actions in the Transition Systems (TS).

#### Known Issues
- When multiple requests are sent in quick succession, none of them may be completed. This is possibly related to the queuing system used.
- Collaboration delay is not functioning correctly, potentially due to an indexing error when checking the plan. If a delay is set, the system will reinitialize, triggering the `unplanned_action` service.

#### Notes
- The `choose_ROI` function needs to be modified based on the specific setup being used. At the moment, multiple implementations are present, and they are commented out based on the specific testing scenario.

### auto_action.py
This node is responsible for completing the actions assigned to the agent. It also manages the synchronization mechanism for collaborative actions.

#### Subscribed Topics
- `next_move_cmd` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    The next move from the output word (action sequence) to be executed by the agent in order to fulfill the plan.

- `synchro_confirm` ([ltl_automaton_msg_srv/msg/SynchroConfirm](/ltl_automaton_msg_srv/msg/SynchroConfirm.msg))

    A confirmation topic common to all agents in the workspace, used to send and receive confirmation messages establishing collaboration between agents.

- `curr_agent/synchro_ready` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    Ready topic of the current agent (`curr_agent`). This topic receives messages from other agents indicating they are ready to perform the requested assistive action.

- `curr_agent/synchro_start` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    Start topic of the current agent (`curr_agent`). This topic receives messages from the requesting agent, allowing `curr_agent` to begin the assistive action.

- Other topics related to manipulation tasks for specific agents in the experimental setup.

- `/qualisys/obstacle/pose` ([geometry_msgs/msg/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

    Topics related to obstacles in the workspace (both agents and other obstacles). These poses are retrieved for the [Model Predictive Controllers (MPC)](/ltl_automaton_synchronization/ltl_automaton_synchronization/waypoint_chaser/) developed.

- `/qualisys/curr_agent/pose` ([geometry_msgs/msg/PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html))

    Topic for retrieving the pose of `curr_agent`, used by the [Model Predictive Controllers (MPC)](/ltl_automaton_synchronization/ltl_automaton_synchronization/waypoint_chaser/).

#### Published Topics
- `ts_state` ([ltl_automaton_msg_srv/msg/TransitionSystemStateStamped](/ltl_automaton_msg_srv/msg/TransitionSystemStateStamped.msg))

    Agent TS state topic, composed of a list of states from the different models within the action model. The planner node receives this topic to update the next action and the set of possible states.

- `agent/synchro_ready` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    Ready topics where `agent` corresponds to the namespaces of agents in the workspace. These topics send a "ready" message to the requesting agent when the agent that initialized the node is ready to perform the assistive action.

- `agent/synchro_start` ([std_msgs/msg/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))

    Start topics where `agent` corresponds to the namespaces of agents in the workspace. These topics send a "start" message to the agents involved in a collaboration, allowing them to begin their actions.

- `curr_agent/cmd_vel` ([geometry_msgs/msg/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))

    Topic used to publish the command velocity generated by the [Model Predictive Controllers (MPC)](/ltl_automaton_synchronization/ltl_automaton_synchronization/waypoint_chaser/) developed for the robots in the lab.

#### Parameters
- `motion_action_dictionary_path` (string, default: '')

    Path to the agent model transition system definition. The file must be generated according to the action and motion models defined in the paper. A script to generate the action and motion dictionary can be found [here](/ltl_automaton_synchronization/ltl_automaton_synchronization/transition_systems/generate_dictionaries_paper.py).

- `obstacles_dictionary_path` (string, default: '')

    Path to the dictionary containing the obstacles in the workspace.
    Each dictionary entry represents an obstacle as a circular region:
    ```Python
    /obstacle_name:
      - 0.0  # x-coordinate
      - 0.0  # y-coordinate
      - 0.3  # radius
    ```

- `agents` (array of strings, default: [''])

    Set of agents in the workspace.

- `dynamic_obstacles` (array of strings, default: [''])

    Indicates which agents in the obstacle dictionary are considered dynamic, meaning their pose must be updated through the motion capture system.

- `is_simulation` (Bool, default: True)

    Flag used to execute simulations. If True, the actions are executed only by waiting the amount of time specified by the TS. If False, all actions are executed by the experimental robots, meaning the MPC controller computes a solution. In this case, it is necessary to be in the lab with all the robots connected, otherwise some errors may occur.

- `execute_manipulation` (Bool, default: False)

    Flag used to control the execution of the manipulation task. If True, the manipulation task is executed (requires being in the lab to work successfully). If False, the manipulation task will be simulated by waiting for a time equal to the cost in the TS.

### manipulation_node
Simple node used to integrate the visual servoing and manipulation functionalities developed for the Rosies in the lab.

## planner
Inside this folder is an updated version of the LTL planner code that includes the ability to execute detours as described in the paper.

## transition_systems
Inside this folder, we have:
- `generate_dictionaries` file used to generate action and motion dictionaries according to the required specifications for using the synchro planner.
- `ts_definitions` file that includes the definitions of the Action model, Motion TS, and Agent TS as described in our paper.

## waypoint_chaser
Inside this folder, you will find the MPC controller with collision avoidance (using control barrier functions) developed for the movement of robots in the lab.