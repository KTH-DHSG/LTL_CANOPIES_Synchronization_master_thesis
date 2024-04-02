#!/usr/bin/env python
import numpy
import rclpy
from  rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from threading import Thread
import sys
import importlib
import yaml
import time
from concurrent.futures import Future

from copy import deepcopy

from std_msgs.msg import String

from ltl_automaton_planner.ltl_tools.ts import TSModel
from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner

import matplotlib.pyplot as plt
import networkx as nx
from ltl_automaton_planner.ltl_automaton_utilities import state_models_from_ts, import_ts_from_file, handle_ts_state_msg

# Import LTL automaton message definitions
from ltl_automaton_messages.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_messages.srv import TaskPlanning, DynamicParameters


def show_automaton(automaton_graph):
    pos=nx.circular_layout(automaton_graph)
    nx.draw(automaton_graph, pos)
    nx.draw_networkx_labels(automaton_graph, pos)
    edge_labels = nx.get_edge_attributes(automaton_graph, 'action')
    nx.draw_networkx_edge_labels(automaton_graph, pos, edge_labels = edge_labels)
    plt.show()
    return

class MainPlanner(Node):
    def __init__(self):
        super().__init__('ltl_planner')
        
        # init parameters, automaton, etc...
        self.init_params()

        self.build_automaton()

        self.setup_pub_sub()

        self.setup_plugins()
        
        # Output plan and first command of plan
        self.publish_possible_states()
        self.publish_plan()
        next_move_msg = String()
        next_move_msg.data = self.ltl_planner.next_move
        self.plan_pub.publish(next_move_msg)

    
    def init_params(self):
        
        #Get parameters from parameter server
        self.agent_name = self.declare_parameter('agent_name', 'agent').value
        self.initial_beta = self.declare_parameter('initial_beta', 1000).value
        self.gamma = self.declare_parameter('gamma', 10).value


        # LTL task
        #----------
        # Get LTL hard task and raise error if not ititialized
        self.hard_task = self.declare_parameter('hard_task', "").value
        if (self.hard_task==""):
            raise RuntimeError("Cannot initialize LTL planner, no hard_task defined")
        # Get LTL soft task (by default initialized as true if not defined)
        self.soft_task = self.declare_parameter('soft_task', "").value

        # Plugin dictionary path
        self.plugin_dictionary_path = self.declare_parameter('plugin_dictionary_path', "").value
        
        # Transition system
        #-------------------
        # Get TS file path from param
        transition_system_path = self.declare_parameter('transition_system_path', "").value
        if transition_system_path == "":
            raise RuntimeError("No Transition System file defined, cannot initialize LTL planner")
        else:
            with open(transition_system_path) as transition_system_textfile:         
                self.transition_system = import_ts_from_file(transition_system_textfile)
        #print(self.transition_system)

        # Parameter if initial TS is set from agent callback or from TS config file
        self.initial_ts_state_from_agent = self.declare_parameter('initial_ts_state_from_agent', False).value

        # define timestap of the first ts_state recived from agent
        # using messages because the structure is easier to use and understand
        self.prev_received_timestamp = rclpy.time.Time().to_msg()
        
        #If initial TS states is from agent, wait from agent state callback
        if self.initial_ts_state_from_agent:
            self.initial_state_ts_dict = None
            self.get_logger().info("LTL planner: waiting for initial TS state from agent to initialize")
            self.initial_state_ts_dict = self.init_ts_state_from_agent(self.wait_for_message("ts_state", TransitionSystemStateStamped))
        else:
            self.initial_state_ts_dict = None


        # In ROS2 all parameters are dynamics and can be changed at runtime with
        # ros2 param set <node_name> <parameter_name> <value>
        # so it is no more needed to have dynamic reconfigure as in ROS1
        # to update also the variable we will use the service DynamicParameters
        self.replan_on_unplanned_move = self.declare_parameter('replan_on_unplanned_move', True).value
        self.check_timestamp =  self.declare_parameter('check_timestamp', True).value

    
    #---------------------------------------------
    # Load plugins from a given plugin dictionary
    #---------------------------------------------
    def load_and_init_plugins(self, plugin_dict):
    # ----
    # Format for plugin dictionary is:
    #   <plugin-name>:
    #       path: <package-path>
    #       args: <additional-argument-dictionary>
    # ----
        self.plugins = {}
        for plugin in plugin_dict:
            # Import plugin module
            try:
                # Import module to a plugin dict
                plugin_module = importlib.import_module(plugin_dict[plugin]["path"])
            except ImportError:
                # Error log message
                self.get_logger().error("LTL planner: Import error on loading plugin %s at %s" % (str(plugin), plugin_dict[plugin]["path"]))
                # Go to next plugin
                break

            # Get plugin class from imported module
            plugin_class = getattr(plugin_module, str(plugin))
            # Create plugin object from class using argument dictionary from parameters
            self.plugins.update({plugin: plugin_class(self, plugin_dict[plugin]['args'])})
            self.get_logger().info("LTL planner: using plugin %s" % str(plugin))

        # Init plugins
        for plugin in self.plugins:
            self.plugins[plugin].init()

     
    def init_ts_state_from_agent(self, msg=TransitionSystemStateStamped):
        initial_state_ts_dict_ = None
        # updating timestamp for the position to avoid replanning at the beginning
        self.prev_received_timestamp = deepcopy(msg.header.stamp)
        # If message is conform (same number of state as number of state dimensions)
        if (len(msg.ts_state.states) == len(msg.ts_state.state_dimension_names)):
            # Create dictionnary with paired dimension_name/state_value
            initial_state_ts_dict_ = dict()
            for i in range(len(msg.ts_state.states)):
                initial_state_ts_dict_.update({msg.ts_state.state_dimension_names[i] : msg.ts_state.states[i]}) 

        # Else message is malformed, raise error
        else:
            self.get_logger.error("LTL planner: received initial states don't match TS state models: "+str(len(msg.ts_state.states))+" initial states and "+str(len(msg.ts_state.state_dimension_names))+" state models")
        
        # Return initial state dictionnary
        return initial_state_ts_dict_


    def build_automaton(self):
        # Import state models from TS
        state_models = state_models_from_ts(self.transition_system, self.initial_state_ts_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = TSModel(self, state_models)
        self.ltl_planner = LTLPlanner(self, self.robot_model, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner.optimal()
        # Get first value from set
        self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

        #show_automaton(self.robot_model)
        #show_automaton(self.ltl_planner.product.graph['buchi'])
        #show_automaton(self.ltl_planner.product)


    def setup_pub_sub(self):
        #IMPORTANTD: all publisher and subscriber must have the same exact qos profile
        # define QoS profile for publishers
        # depth and history are used to set a quiesize of 1 while durability is to store messages for late subscribers
        self.qos_profile = rclpy.qos.QoSProfile(depth=1, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # Prefix plan publisher
        
        self.prefix_plan_pub = self.create_publisher(LTLPlan, 'prefix_plan', self.qos_profile)

        # Suffix plan publisher
        self.suffix_plan_pub = self.create_publisher(LTLPlan, 'suffix_plan', self.qos_profile)

        # Possible states publisher
        self.possible_states_pub = self.create_publisher(LTLStateArray, 'possible_ltl_states', self.qos_profile)

        # Initialize subscriber to provide current state of robot
        self.state_sub = self.create_subscription(TransitionSystemStateStamped, 'ts_state', self.ltl_state_callback, self.qos_profile) 

        # Initialize publisher to send plan commands
        self.plan_pub = self.create_publisher(String, 'next_move_cmd', self.qos_profile) 

        # Initialize task replanning service
        self.replan_srv = self.create_service(TaskPlanning, 'replanning', self.task_replanning_callback)

        # Initialize dynamic parameter service
        self.dynparam_srv = self.create_service(DynamicParameters, 'update_parameters', self.dynparam_callback)


    def setup_plugins(self):
        # Get plugin dictionary path from parameters
        if self.plugin_dictionary_path == "":
            self.plugins = {}
        else:
            # Get plugin dictionary from file path
            with open(self.plugin_dictionary_path) as plugin_dict_file:
                plugin_dict = yaml.safe_load(plugin_dict_file)
            
            # If plugins are specified, try to load them
            self.load_and_init_plugins(plugin_dict)

            # Setup plugin subscribers and publisers
            for plugin in self.plugins:
                self.plugins[plugin].set_sub_and_pub()

    
    def dynparam_callback(self, config, response):
        self.replan_on_unplanned_move = config.replan_on_unplanned_move
        self.check_timestamp = config.check_timestamp
        replan_on_unplanned_move_parameter = Parameter('replan_on_unplanned_move', Parameter.Type.BOOL, config.replan_on_unplanned_move)
        check_timestamp_parameter = Parameter('check_timestamp', Parameter.Type.BOOL, config.check_timestamp)
        self.set_parameters([replan_on_unplanned_move_parameter, check_timestamp_parameter])
        response.success = True
        return response
    
    
    
    def task_replanning_callback(self, task_planning_req, response):
        # Extract task specification from request
        hard_task = task_planning_req.hard_task
        soft_task = task_planning_req.soft_task
        # Replan and return success status
        response.success = self.ltl_planner.replan_task(hard_task, soft_task, self.ltl_planner.curr_ts_state)
        # If successful, change parameters
        if response.success:
            hard_task_parameter = Parameter('hard_task', Parameter.Type.STRING, hard_task)
            soft_task_parameter = Parameter('soft_task', Parameter.Type.STRING, soft_task)
            self.set_parameters([hard_task_parameter, soft_task_parameter])
        # Return response
        return response

    def ltl_state_callback(self, msg=TransitionSystemStateStamped()):
        # Extract TS state from message
        state = handle_ts_state_msg(msg.ts_state)
        
        #-------------------------
        # Check if state is in TS
        #-------------------------
        if (state in self.robot_model.nodes()):
            
            # If timestamp check is enabled, check the timestamp
            if not (self.check_timestamp and (msg.header.stamp.sec == self.prev_received_timestamp.sec)):
                # Update previously received timestamp
                self.prev_received_timestamp = deepcopy(msg.header.stamp)
                
                # Update current state
                self.ltl_planner.curr_ts_state = state

                #-----------------------------------------------------------------------
                # Try update possible state and if error (forbidden transition), replan
                #-----------------------------------------------------------------------
                if not self.ltl_planner.update_possible_states(state):
                    self.get_logger().error('Can not update possible states - forbidden transition, replanning...')

                    # Replan
                    self.ltl_planner.replan_from_ts_state(state)
                    self.publish_plan()
                    
                    # Publish next move
                    self.get_logger().warning('LTL planner: error in possible states, replanning done and publishing next move')
                    next_move_msg = String()
                    next_move_msg.data = self.ltl_planner.next_move
                    self.plan_pub.publish(next_move_msg)

                    return

                # Publish possible states
                self.publish_possible_states()

                #--------------------------
                # Manage next move in plan
                #--------------------------
                # If state is next state in plan, find next_move and output
                if self.is_next_state_in_plan(state):
                    self.ltl_planner.find_next_move()

                    # Publish next move
                    self.get_logger().info('LTL planner: Publishing next move')
                    next_move_msg = String()
                    next_move_msg.data = self.ltl_planner.next_move
                    self.plan_pub.publish(next_move_msg)

                # If state is not the next one in plan replan 
                elif self.replan_on_unplanned_move:
                    self.get_logger().warning('LTL planner: Received state is not the next one in the plan, replanning and publishing next move')
                    # Replan with state as initial
                    self.ltl_planner.replan_from_ts_state(state)
                    self.publish_plan()

                    # Publish next move
                    next_move_msg = String()
                    next_move_msg.data = self.ltl_planner.next_move
                    self.plan_pub.publish(next_move_msg)

                #-------------
                # Run plugins
                #-------------
                for plugin in self.plugins:
                    self.plugins[plugin].run_at_ts_update(state)

            # If timestamp is indentical to previoulsy received message and parameters "check_timestamp" is true
            else:
                self.get_logger().warning("LTL planner: not updating with received TS state %s, timestamp identical to previously received message timestamp at time %f" % (str(state), self.prev_received_timestamp.sec))

        #--------------------------------------------
        # If state not part of the transition system
        #--------------------------------------------
        #FIXMED: using none as a error
        else:
            #ERROR: unknown state (not part of TS)
            next_move_msg = String()
            next_move_msg.data = 'None'
            self.plan_pub.publish(next_move_msg)
            self.get_logger().warning('State is not in TS plan!')

    #-----------------------------------------------------
    # Check if given TS state is the next one in the plan
    #-----------------------------------------------------
    def is_next_state_in_plan(self, ts_state):
        # Check if plan is in prefix phase
        if self.ltl_planner.segment == 'line':
            # Check if state is the next state of the plan
            if ts_state == self.ltl_planner.run.line[self.ltl_planner.index+1]:
                return True
        # Check if plan is in suffix phase
        elif self.ltl_planner.segment == 'loop':
            # Check if state is the next state of the plan
            if ts_state == self.ltl_planner.run.loop[self.ltl_planner.index+1]:
                return True
        return False

    #----------------------------------------------
    # Publish prefix and suffix plans from planner
    #----------------------------------------------
    def publish_plan(self):
        # If plan exists
        if not (self.ltl_planner.run == None):
            # Prefix plan
            #-------------
            prefix_plan_msg = LTLPlan()
            prefix_plan_msg.header.stamp = self.get_clock().now().to_msg()
            prefix_plan_msg.action_sequence = self.ltl_planner.run.pre_plan
            # Go through all TS state in plan and add it as TransitionSystemState message
            for ts_state in self.ltl_planner.run.line:
                ts_state_msg = TransitionSystemState()
                for set in self.ltl_planner.product.graph['ts'].graph['ts_state_format']:
                    ts_state_msg.state_dimension_names.append(set[0]) 
                #ts_state_msg.state_dimension_names = self.ltl_planner.product.graph['ts'].graph['ts_state_format']
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]
                # Add to plan TS state sequence
                prefix_plan_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            self.prefix_plan_pub.publish(prefix_plan_msg)

            # Suffix plan
            #-------------
            suffix_plan_msg = LTLPlan()
            suffix_plan_msg.header.stamp = self.get_clock().now().to_msg()
            suffix_plan_msg.action_sequence = self.ltl_planner.run.suf_plan
            # Go through all TS state in plan and add it as TransitionSystemState message
            for ts_state in self.ltl_planner.run.loop:
                ts_state_msg = TransitionSystemState()
                
                for set in self.ltl_planner.product.graph['ts'].graph['ts_state_format']:
                    ts_state_msg.state_dimension_names.append(set[0])  
                
                #ts_state_msg.state_dimension_names = self.ltl_planner.product.graph['ts'].graph['ts_state_format']
                # If TS state is more than 1 dimension (is a tuple)
                if type(ts_state) is tuple:
                    ts_state_msg.states = list(ts_state)
                # Else state is a single string
                else:
                    ts_state_msg.states = [ts_state]

                # Add to plan TS state sequence
                suffix_plan_msg.ts_state_sequence.append(ts_state_msg)

            # Publish
            self.suffix_plan_pub.publish(suffix_plan_msg)

    #-------------------------
    # Publish possible states
    #-------------------------
    def publish_possible_states(self):
        # Create message
        possible_states_msg = LTLStateArray()
        # For all possible state, add to the message list
        for ltl_state in self.ltl_planner.product.possible_states:
            ltl_state_msg = LTLState()
            # If TS state is more than 1 dimension (is a tuple)
            if type(ltl_state[0]) is tuple:
                ltl_state_msg.ts_state.states = list(ltl_state[0])
            # Else state is a single string
            else:
                ltl_state_msg.ts_state.states = [ltl_state[0]]
            # Add state dimension names
            for set in self.ltl_planner.product.graph['ts'].graph['ts_state_format']:
                ltl_state_msg.ts_state.state_dimension_names.append(set[0])           

            #ltl_state_msg.ts_state.state_dimension_names = self.ltl_planner.product.graph['ts'].graph['ts_state_format']
            ltl_state_msg.buchi_state = str(ltl_state[1])
            possible_states_msg.ltl_states.append(ltl_state_msg)

        # Publish
        self.possible_states_pub.publish(possible_states_msg)
    

    #-------------------------
    # wait_for_message
    #------------------------- 
    # this function is used to replicate the behaviour or rospy.wait_for_message present in ROS1        
    def wait_for_message(self, topic_name, message_type):
        
        self.get_logger().info('Waiting for message on topic: %s' % topic_name)

        # Create a Future to wait for the message
        future = Future()

        # Define a callback function to be called when a message is received
        def callback(msg):
            self.get_logger().info('Received message on topic: %s' % topic_name)
            future.set_result(msg)

        # Subscribe to the topic
        self.create_subscription(message_type, topic_name, callback, self.qos_profile) 
        # Wait for the message
        rclpy.spin_until_future_complete(self, future)
        
        # Return the message content
        return future.result()
    
def spin_srv(executor):
    try: 
        executor.spin()
    except rclpy.executors.ExternalShutdownExeption:
        pass
    

#==============================
#             Main
#==============================
def main():
    rclpy.init()    
    ltl_planner_node = MainPlanner()  
    executor = SingleThreadedExecutor() 
    executor.add_node(ltl_planner_node)
    #thread = Thread(target=spin_srv, args=(executor,), daemon=True)
    executor.spin() 
    #thread.start()
    #rclpy.spin(ltl_planner_node)



if __name__ == '__main__':
    main()