# name of the agent sending the reply
string agent

# list of assistive actions to be executed
string[] actions

# list of regions of interest where the actions will be executed
string[]  roi

# list of booleans indicating if the agent is available to execute the action
bool[]  available

# time to execute the actions
float32[]  time