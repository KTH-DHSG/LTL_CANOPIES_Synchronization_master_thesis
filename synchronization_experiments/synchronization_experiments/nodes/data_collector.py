import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ltl_automaton_msg_srv.msg import SynchroConfirm, SynchroRequest, SynchroReply
from std_msgs.msg import String
from random import shuffle, choice, uniform
from gurobipy import *
from collections import defaultdict
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
import pickle
import yaml
class AgentData:
    def __init__(self):
        self.action = []
        self.action_type = []
        self.action_start = []
        self.collab_start = []

    def add_action(self, action, action_type, action_start):
        self.action.append(action)
        self.action_type.append(action_type)
        self.action_start.append(action_start)
    
    def add_collab(self, collab_start):
        self.collab_start.append(collab_start)




class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.init_params()
        self.setup_pub_sub()            
        self.timer_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.save_timer = self.create_timer(10, self.save_callback, callback_group=self.timer_callback_group)
        self.init_time = self.get_clock().now().to_msg().sec+self.get_clock().now().to_msg().nanosec*1e-9  
            
            
            
    def init_params(self):
        
        #getting save location for data
        self.save_location = self.declare_parameter('save_location', 'data_collector.pkl').value
        
        # getting all the agents involved
        agents = self.declare_parameter('agents', ['']).value 
        agents_types = self.declare_parameter('agents_types', ['']).value
        
        # define agents and types
        self.agents={}
        for i in range(len(agents)):
            self.agents[agents[i]]=agents_types[i]
        print(self.agents)
        # getting type of agents involved THE FOLLOWING 2 PARAMETERS MUST HAVE THE SAME ORDER
        self.possible_agent_types = self.declare_parameter('possible_agent_types', ['']).value
        # retrieving full dictionary and retrieving motion and action dictionaries        
        motion_action_dictionary_paths = self.declare_parameter('motion_action_dictionary_paths', ['']).value
        # building all the action disctionarie for each type of robot
        self.actions_dicts = {}
        for i in range(len(self.possible_agent_types)):        
            with open(motion_action_dictionary_paths[i], 'r') as file:
                motion_action_dictionary = yaml.safe_load(file)
                motion_dict = motion_action_dictionary['motion']
                action_dict = motion_action_dictionary['action']
                # motion and actions
                motion_ts = MotionTS(motion_dict)
                action_mod = ActionModel(action_dict)        
                # Here we take the product of each element of state_models to define the full TS
                robot_model = MotActTS(motion_ts, action_mod)
                robot_model.build_full()
                self.actions_dicts[self.possible_agent_types[i]] = robot_model.graph['action'].action

        #setting up data to be saved using only names of agents
        self.data_collector = {} 
        for agent in agents:
            self.data_collector[agent] = AgentData()              
    
    def setup_pub_sub(self):
        self.qos_profile = rclpy.qos.QoSProfile(depth=len(self.agents), history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        cb_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # Initialize subscriber to recieve the actions done by each agent
        for agent in self.agents.keys():
            self.action_sub = self.create_subscription(String, agent+'/next_move_cmd', lambda msg, ag=agent: self.update_actions_cb(msg, ag), self.qos_profile, callback_group=cb_group)
            self.synchro_start_sub = self.create_subscription(String, agent+'/synchro_start', lambda msg, ag=agent: self.update_collab_cb(msg, ag), self.qos_profile)
       
        
    def save_callback(self):
        with open(self.save_location, 'wb') as f:
            pickle.dump(self.data_collector, f, pickle.HIGHEST_PROTOCOL)
        message = 'Data saved at: '+str(self.save_location)
        self.get_logger().info(f"\033[32m{message}\033[0m")
    
    
    def update_actions_cb(self, msg, agent):
        time= self.get_clock().now().to_msg()
        action_start = time.sec+time.nanosec*1e-9-self.init_time
        action = msg.data
        # added because different label and entry in the dictionary
        if action=='none':
            action_type = 'local'
        else:
            action_type = self.actions_dicts[self.agents[agent]][action]['type']        
        self.data_collector[agent].add_action(action, action_type, action_start)
        self.get_logger().info('[Action Recorded] Agent: '+str(agent)+ ' Action: '+str(action)+ ' Type: '+str(action_type)+ ' Time: ' + str(action_start))   
        
    def update_collab_cb(self, msg, agent):
        time= self.get_clock().now().to_msg()
        collab_start = time.sec+time.nanosec*1e-9-self.init_time
        self.data_collector[agent].add_collab(collab_start)
        message ='[Collaboration Recorded] Agent: '+str(agent)+' Collaboration Start: '+str(collab_start)
        self.get_logger().info(f"\033[33m{message}\033[0m")        
        #if the data of the master is not empty
        if self.data_collector[msg.data].collab_start: 
            # if the last data recorded is more than 2 second ago
            # (just to avoid double recording of the same data)
            # then add the data
            if collab_start-self.data_collector[msg.data].collab_start[-1]>2:
                self.data_collector[msg.data].add_collab(collab_start)
                message ='[Collaboration Recorded] Agent: '+str(msg.data)+' Collaboration Start: '+str(collab_start)
                self.get_logger().info(f"\033[33m{message}\033[0m")
        # if empty then just add the data
        else:
            self.data_collector[msg.data].add_collab(collab_start)
            message ='Recorded Collaboration Agent: '+str(msg.data)+' Collaboration Start: '+str(collab_start)
            self.get_logger().info(f"\033[33m{message}\033[0m")
    
    








#==============================
#             Main
#==============================
def main():
    rclpy.init()   
    node = DataCollector() 
    executor = MultiThreadedExecutor() 
    executor.add_node(node)
    executor.spin()



if __name__ == '__main__':
    main()

