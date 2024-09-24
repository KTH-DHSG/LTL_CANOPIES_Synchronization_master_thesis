# Transition system definition
The final transition system is built from one or more action models. Those action models are also transition systems and doing their cartesian graph product gives the final transition system.

As the final transition system, each individual action model transition system is discrete, finite, and deterministic. The input on each transition is an action that can be carried out by the agent.

The number of individual transition systems used to build the FTS is not limited, but the growing complexity of adding transition models can impact performance prohibitively. Each final transition system state is a combination (as a list) of state from each individual transition system (TS1_state, TS2_state,...).

In addition, each action (transition) had a guard formula. A guard formula is an LTL expression that will evaluated against the node label when building the final transition system's transitions. The guard formula can therefor be used to restrict some action to being only available from some nodes. For example in a transition system built from a region transition system with a "r5" node, and a load transition system with action "drop", using the guard "r5" for the action "drop" would only make this transition available at region "r5".

## Example 
As an example, a final transition system could be the product of a transition system describing the agent position as a named 2D region with a transition system describing the agent load state as either loaded or unloaded. The transition between each state would then be either a "go to region" action or a "pick/drop" action.

<a href="url"><img src="https://github.com/KTH-SML/ltl_automaton_core/blob/main/documentation/pictures/fts_build_example.jpg" align="center" height="990" width="700"/></a>

**Example transition system built from two transition systems**

A series of standard transition systems (2D regions,...) are already provided by the package [ltl_automaton_std_transition_systems](../blob/main/ltl_automaton_std_transition_systems).

## Writing the transition system config file
The transition system textfile has to be loaded as a string to the ROS parameter server (the same way as a robot description textfile). To do so, the following line needs to be included in the launch file (replacing the example file location by the proper transition system file):

```XML
  <!-- Transition system -->
  <param name="transition_system_textfile" type="str" textfile="$(find ltl_automaton_planner)/config/example_ts.yaml" />
```

The transition system textfile is YAML file with a nested dictionary structure. The structure is as follow:

```Python
# Array of state model names
state_dim: [model1, model2, ...]

# individual state models, or action models
state_models:
    # action model name
    model1:
        ts_type:  #model type
        # initial states
        initial: "node1"
        nodes:
            node1:
                # attributes dictionary, used by the agent node but not the planner node (contain additional information on the state, format can vary
                attr: ...
                # connection dictionary, list all connected states and the input action on the transition
                connected_to:
                    node2: "action1"
            node2
            ...
    model2:
        ts_type: ...
        initial: ...
        nodes: ...
    ...

# Actions or input (transitions) dictionary
actions:
    # action name
    action1:
        # action type, not used by planner but by the agent node
        type: "move"
        weight: 10
        guard: "1" # guard formula, leave to "1" for the action to be available at all node, or use LTL formula to restrict it
        # attributes dictionary, used by the agent node but not the planner node (contain additional information on the state, format can vary
        attr: ...
    ...


```