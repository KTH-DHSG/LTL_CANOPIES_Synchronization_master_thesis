#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from  rclpy.callback_groups import ReentrantCallbackGroup
from copy import deepcopy
from std_msgs.msg import Bool

# Import transition system loader
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file

#Import LTL automaton message definitions
from ltl_automaton_messages.msg import TransitionSystemStateStamped, TransitionSystemState
from ltl_automaton_messages.srv import TrapCheck

#=============================================
#       Mix Initative Controller object
#            for action commands
#                     ---
#  Take as input an action name (String msg)
#  to test the potential resulting state as
#   trap or not and output either planner or
#           human action command
#=============================================
class BoolCmdMixer(Node):
    def __init__(self):
        super().__init__('bool_mic_hil_controller')
        
        # Get parameters
        self.load_params()

        # Setup subscribers and publishers
        self.set_pub_sub()

        self.get_logger().info("Human-in-the-loop boolean commands mix-initiative controller initialized")

    #--------------------------------------------------
    # Load controller parameters
    #--------------------------------------------------   
    def load_params(self):

        self.curr_ts_state = None

        # Get TS from param
        transition_system_path = self.declare_parameter('transition_system_path', "").value
        if transition_system_path == "":
            raise RuntimeError("No Transition System file defined, cannot initialize LTL planner")
        else:
            with open(transition_system_path) as transition_system_textfile:         
                self.transition_system = import_ts_from_file(transition_system_textfile)

        # Get monitored TS state model
        self.state_dimension_name = self.declare_parameter("state_dimension_name", "load").value

        # Get monitored action
        self.monitored_action = self.declare_parameter("monitored_action", "pick").value
        
        # Create dict to retrieve next state given current state and next action
        self.action_to_state = dict()
        for state in self.transition_system['state_models'][self.state_dimension_name]['nodes']:
            temp_dict = dict()
            for connected_state in self.transition_system['state_models'][self.state_dimension_name]['nodes'][state]['connected_to']:
                temp_dict.update({self.transition_system['state_models'][self.state_dimension_name]['nodes'][state]['connected_to'][connected_state]: connected_state})
            self.action_to_state.update({state: temp_dict})

    def set_pub_sub(self):
        """
        Setup subscribers, publishers and service clients.
        """
        # define QoS profile for publishers
        # depth and history are used to set a quiesize of 1 while durability is to store messages for late subscribers
        self.qos_profile = rclpy.qos.QoSProfile(depth=1, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.callback_group = ReentrantCallbackGroup() 

        # Set trap check service client
        self.trap_cheq_srv = self.create_client(TrapCheck, "check_for_trap", callback_group=self.callback_group)

        # Set mix initiave controller output
        self.mix_cmd_pub = self.create_publisher(Bool, "mix_cmd",  self.qos_profile)

        # Set agent TS state subscriber
        self.ts_state_sub = self.create_subscription(TransitionSystemStateStamped, "ts_state",  self.ts_state_callback, self.qos_profile)

        # Set human input planner
        self.key_cmd_sub = self.create_subscription(Bool, "key_cmd", self.teleop_cmd_callback, self.qos_profile, callback_group=self.callback_group)

        # Set planner input subscriber
        self.planner_cmd_sub = self.create_subscription(Bool, "planner_cmd", self.planner_cmd_callback, self.qos_profile)


    def ts_state_callback(self, msg=TransitionSystemStateStamped()):
        """
        Agent TS state callback.
        """
        
        # If not using same state model type, print warning and ignore message
        if not self.state_dimension_name in msg.ts_state.state_dimension_names:
            self.get_logger().debug("Received TS state does not include state model type used by boolean command HIL MIC (%s), TS state is of type %s"
                          % (self.state_dimension_name, msg.ts_state.state_dimension_names))
        # If lenght of states is different from length of state dimension names, message is malformed
        # print warning and ignore message
        if not (len(msg.ts_state.state_dimension_names) == len(msg.ts_state.states)):
            self.get_logger().debug("Received TS state but number of states: %i doesn't correpond to number of state dimensions: %i"
                          % (len(msg.ts_state.states),len(msg.ts_state.state_dimension_names)))
        # Else message is valid, save it
        else:
            self.curr_ts_state = msg.ts_state



    def planner_cmd_callback(self, msg):
        """
        Planner command input callback.
        """
        
        # Use planner input directly
        self.mix_cmd_pub.publish(msg)


    def teleop_cmd_callback(self, msg=Bool()):
        """
        Human command input callback
        """
        # If boolean command is true
        if msg.data:
            # Get next state if action is executed
            for i in range(len(self.curr_ts_state.state_dimension_names)):
                # Find state in the TS state
                if self.curr_ts_state.state_dimension_names[i] == self.state_dimension_name:
                    # Check if trap using potential state if action would to be executed
                    #IMPORTANTD: if the monitored action is not possible then there is an error ask if normal or needs to be fixed
                    # like r1 loaded  and monitored action is pick
                    #self.get_logger().warning(str(self.action_to_state[self.curr_ts_state.states[i]]))
                    if not self.check_for_trap(self.action_to_state[self.curr_ts_state.states[i]][self.monitored_action]):
                        # If not a trap publish command
                        self.mix_cmd_pub.publish(msg)

            # If TS state doesn't contain proper dimension
            return None

    def check_for_trap(self, potential_state):
        """
        Check if risk of trap when executing commands.
        """
        # Create check for trap request from TS state
        ts_state_to_check = deepcopy(self.curr_ts_state)
        for i in range(len(self.curr_ts_state.state_dimension_names)):
            # Replace current region by region to check
            if self.curr_ts_state.state_dimension_names[i] == self.state_dimension_name:
                ts_state_to_check.states[i] = potential_state

                # Populate request and call service to check if closest region would trigger a trap state
                check_for_trap_req = TrapCheck.Request()
                check_for_trap_req.ts_state = ts_state_to_check
                
                # Call service to check if a state is a trap
                self.trap_cheq_srv.wait_for_service()
                check_for_trap_res = self.trap_cheq_srv.call(check_for_trap_req)

                self.get_logger().warning("LTL boolean command MIC: testing next state %s" % (ts_state_to_check.states))

                # if either unconnected or a trap (both considered a trap)
                if not check_for_trap_res.is_connected or (check_for_trap_res.is_connected and check_for_trap_res.is_trap):
                    self.get_logger().warning("LTL boolean command MIC: Agent is in state %s and state %s is a trap" % (self.curr_ts_state.states, check_for_trap_req.ts_state.states))
                    # Return true if a trap
                    return True
        # If not a trap or cannot be tested
        return False


#============================
#            Main            
#============================
def main():
    rclpy.init()        
    try: 
        bool_mic_node = BoolCmdMixer()
        executor = MultiThreadedExecutor() 
        executor.add_node(bool_mic_node) 
        executor.spin()        
    except ValueError as e:
        rclpy.logging.get_logger("bool_cmd_hil_mic").error("Boolean Command HIL MIC: %s" %(e))
        sys.exit(0)


if __name__ == '__main__':
    main()