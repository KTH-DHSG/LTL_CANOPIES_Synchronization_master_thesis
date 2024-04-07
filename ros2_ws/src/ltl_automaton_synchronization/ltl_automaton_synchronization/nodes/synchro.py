from ltl_automaton_messages.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLStateArray
from ltl_automaton_messages.srv import ClosestState, TaskPlanning
from ltl_automaton_planner.ltl_tools.buchi import *
from ltl_automaton_planner.ltl_tools.discrete_plan import *
from ltl_automaton_planner.ltl_tools.ltl_planner import *
from ltl_automaton_planner.ltl_tools.ltl2ba import *
from ltl_automaton_planner.ltl_tools.product import *
from ltl_automaton_planner.ltl_tools.promela import *
from ltl_automaton_planner.ltl_tools.ts import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Header, Bool
import os
from rclpy.executors import MultiThreadedExecutor 
from ltl_automaton_planner.nodes.planner_node import show_automaton
from ament_index_python.packages import get_package_prefix
import yaml
from ltl_automaton_planner.nodes.planner_node import MainPlanner

from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS

import matplotlib.pyplot as plt
import rclpy
import sys

class SynchroPlanner(MainPlanner):
    def __init__(self):
        super().__init__()
        
    def init_params(self):
        
        # retrieving full dictionary and retrieving motion and action dictionaries        
        motion_action_dictionary_path = self.declare_parameter('motion_action_dictionary_path', '').value
        with open(motion_action_dictionary_path, 'r') as file:
            motion_action_dictionary = yaml.safe_load(file)
            self.motion_dict = motion_action_dictionary['motion']
            self.action_dict = motion_action_dictionary['action']
            
        self.declare_parameter('transition_system_path', "none")
        
        super().init_params()
    
    
    def build_automaton(self):
        
        # updates the initial states of the dictionaries
        self.set_initial_state()
        
        # motion and actions
        motion_ts = MotionTS(self.motion_dict)
        action_mod = ActionModel(self.action_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = MotActTS(motion_ts, action_mod)
        
        # initialize the planner
        self.ltl_planner = LTLPlanner(self, self.robot_model, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner.optimal()
        # Get first value from set
        self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])

    def set_initial_state(self):        
        # if None then the initial state will be taken direcly from the dictionaries
        if not self.initial_state_ts_dict == None:
            # otherwise we modify the dictionaries to the state given 
            self.motion_dict['initial'] =  self.initial_state_ts_dict[self.motion_dict['type']]
            self.action_dict['initial'] =  self.initial_state_ts_dict[self.action_dict['type']]


#==============================
#             Main
#==============================
def main():
    rclpy.init()   
    ltl_planner_node = SynchroPlanner() 
    executor = MultiThreadedExecutor() 
    executor.add_node(ltl_planner_node)
    executor.spin()



if __name__ == '__main__':
    main()
    
