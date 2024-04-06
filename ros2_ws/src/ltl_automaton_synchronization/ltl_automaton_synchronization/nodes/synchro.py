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
        motion_dictionary_path = self.declare_parameter('motion_dictionary_path', '').value
        with open(motion_dictionary_path, 'r') as file:
            self.motion_dict = yaml.safe_load(file)       
        
        action_dictionary_path = self.declare_parameter('action_dictionary_path', '').value
        with open(action_dictionary_path, 'r') as file:
            self.action_dict = yaml.safe_load(file)
            
        self.declare_parameter('transition_system_path', "none")
        
        super().init_params()
    
    
    def build_automaton(self):
        
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
    
