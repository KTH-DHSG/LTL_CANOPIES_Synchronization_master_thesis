import rclpy
from rclpy.executors import MultiThreadedExecutor

from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner
from ltl_automaton_planner.nodes.planner_node import MainPlanner
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
from ltl_automaton_synchronization.planner.synchro_planner import sync_LTLPlanner


from ltl_automaton_planner.ltl_tools.discrete_plan import compute_path_from_pre, dijkstra_targets

import networkx as nx
import matplotlib.pyplot as plt
import yaml

# auxiliary function to show the automaton
def show_automaton(automaton_graph, edge_label='action'):
    pos = nx.circular_layout(automaton_graph)
    fig, axs = plt.subplots(1,1,figsize=(100, 50))
    nx.draw_networkx_nodes(automaton_graph, pos, ax=axs, node_size=500, node_color="#acddc5", edgecolors='g')
    nx.draw_networkx_labels(automaton_graph, pos, ax=axs, font_weight='bold', font_size=12)
    nx.draw_networkx_edges(automaton_graph, pos, ax=axs, edgelist=automaton_graph.edges(data=True), edge_color="r")#, connectionstyle="arc3,rad=0.1")
    nx.draw_networkx_edge_labels(automaton_graph, pos, ax=axs, edge_labels={(u, v): d[edge_label] for u, v, d in automaton_graph.edges(data=True)}, label_pos=0.66)
    plt.show()
    return

class SynchroPlanner(MainPlanner):
    def __init__(self):
        super().__init__()
        self.agent_name = self.get_namespace()
        
        
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
        self.ltl_planner = sync_LTLPlanner(self, self.robot_model, self.hard_task, self.soft_task, self.initial_beta, self.gamma)
        self.ltl_planner.optimal()
        # Get first value from set
        self.ltl_planner.curr_ts_state = list(self.ltl_planner.product.graph['ts'].graph['initial'])[0]

        # initialize storage of set of possible runs in product
        self.ltl_planner.posb_runs = set([(n,) for n in self.ltl_planner.product.graph['initial']])
        
        #print(self.ltl_planner.action_dictionary)
        #self.ltl_planner.index = 2
        request = self.ltl_planner.cooperative_action_in_horizon(30, self.chose_ROI)

        
        print('request: %s' % request)
        print('run_line: %s' % self.ltl_planner.run.line)
        print('run_loop: %s' % self.ltl_planner.run.loop)
        print ('run_pre_plan: %s' % self.ltl_planner.run.pre_plan)
        print ('run_pre_plan_cost: %s' % self.ltl_planner.run.pre_plan_cost)
        
        self.ltl_planner.index = 0
        
        reply1 = self.ltl_planner.evaluate_request(request)
        print('reply: %s' % reply1) 
        print('paths: %s' % self.ltl_planner.path)

        reply2 = reply1.copy()
        reply2[('r1','h1')] = (True, 50)
        reply2[('r2','a2')] = (True, 20)

        reply_dict={'/agent1': reply1, '/agent2': reply2, '/agent3': reply1}
        
        
        confirm, time = self.ltl_planner.confirmation(request, reply_dict)
        print('confirm: %s' % confirm)
        print('time: %s' % time)
        print(self.get_namespace())
        print(confirm[self.get_namespace()])
        
        
        self.ltl_planner.adapt_plan(('r2', 'a2'), time) 
        print(self.ltl_planner.detour_path)
        print (self.ltl_planner.detour)
        '''
        
        
        
        print('request: %s' % request)
        print('run_line: %s' % self.ltl_planner.run.line)
        print('run_loop: %s' % self.ltl_planner.run.loop)
        print ('run_pre_plan: %s' % self.ltl_planner.run.pre_plan)
        print ('run_pre_plan_cost: %s' % self.ltl_planner.run.pre_plan_cost)
        j=0
        k=0
        while j<len(self.ltl_planner.run.pre_plan_cost)-1:
            k = k+self.ltl_planner.run.pre_plan_cost[j]
            print (k)
            print (j)
            print('action: %s' % self.ltl_planner.run.pre_plan[j])
            j += 1
        print('k: %s' % k)
        if request: 
            reply = self.ltl_planner.evaluate_request(request)
            print('reply: %s' % reply)
            detour = self.ltl_planner.path[('r2','a2')]
            print('detour: %s' % detour)
        states = self.ltl_planner.run.line
        print('states: %s' % states)
        #fist_action_state
        #for i in len(states):
            
        #print(self.ltl_planner.product.graph['accept_with_cycle'])
        #print(self.ltl_planner.product.graph['accept'])
        #print(self.ltl_planner.product.graph['initial'])
        
        next_states= []
        set_list = list(self.ltl_planner.product.graph['initial'])
        next_move = self.ltl_planner.next_move
        curr_node = set_list[0]
        print (curr_node)
        targets=[]
        for node in self.ltl_planner.product.graph['accept']:
            if node[0]==('r2','a2'):
                targets.append(node)
        #print(targets)
        #result, dist= dijkstra_targets(self.ltl_planner.product, curr_node, targets)
        #print('yuppy')
        #print(dist)
        #for value in result:
                #print(value)
        #print('yuppy')
        #print(next_move)
        for node in self.ltl_planner.product.successors(curr_node):
            if (self.ltl_planner.product[curr_node][node]['action'])==next_move:
                next_states.append(node)        
        #print(next_states)    
        curr_node = next_states[-1]
        next_move= self.ltl_planner.run.pre_plan[1]
        #print (curr_node) 
        #print (next_move) 
        for node in self.ltl_planner.product.successors(curr_node):
            #print(node)
            if (self.ltl_planner.product[curr_node][node]['action'])==next_move:
                next_states.append(node)   
       # print(next_states)
        curr_node = next_states[-1]
        #print (curr_node)
        #show_automaton(self.robot_model, 'action')
        #print (self.robot_model.graph['action'].action)
        #for item in self.ltl_planner.product.edges(data=True):
            #print(item)
        #show_automaton(self.ltl_planner.product.graph['buchi'], 'guard_formula')
        #show_automaton(self.ltl_planner.product, 'action')
    '''
    
    # returns just the first region for simplicity for now
    def chose_ROI(self, dependency, *args, **kwargs):
        return dependency[1][0]  
        

    def set_initial_state(self):        
        # if None then the initial state will be taken direcly from the dictionaries
        if not self.initial_state_ts_dict == None:
            # otherwise we modify the dictionaries to the state given 
            self.motion_dict['initial'] =  self.initial_state_ts_dict[self.motion_dict['type']]
            self.action_dict['initial'] =  self.initial_state_ts_dict[self.action_dict['type']]




    #-----------------------------------------------------
    # Check if given TS state is the next one in the plan
    #-----------------------------------------------------
    def is_next_state_in_plan(self, ts_state):
        # check if the next state is in the detour path
        if self.ltl_planner.segment == 'detour':
            if ts_state == self.ltl_planner.detour_path[self.ltl_planner.dindex+1]:
                return True
        else:
            # calls the origina function
            super().is_next_state_in_plan(ts_state)
        
'''

 add a callback with timer that every horizon seconds checks  if there is a cooperative action in the plan
 
 if there is then it sends a request (message in topic)



'''
















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
    
