import rclpy
from rclpy.executors import MultiThreadedExecutor

from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner
from ltl_automaton_planner.nodes.planner_node import MainPlanner
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
from ltl_automaton_synchronization.planner.synchro_planner import sync_LTLPlanner
from  rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from ltl_automaton_messages.msg import SynchroConfirm, SynchroRequest, SynchroReply
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
        self.timer_callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.timer_callback_group)
        self.starting_time=0
        
    def init_params(self):
                
        # retrieving full dictionary and retrieving motion and action dictionaries        
        motion_action_dictionary_path = self.declare_parameter('motion_action_dictionary_path', '').value
        with open(motion_action_dictionary_path, 'r') as file:
            motion_action_dictionary = yaml.safe_load(file)
            self.motion_dict = motion_action_dictionary['motion']
            self.action_dict = motion_action_dictionary['action']
            
        # force the parameter to avoid compatibility errors with the original planner
        self.declare_parameter('transition_system_path', "none")
        
        # initializing the original planner
        super().init_params()
        
        # updating the agent name
        self.agent_name = self.get_namespace()
        
        # getting all the agents involved
        self.agents = self.declare_parameter('agents', []).value 
        # removing the current agent from the list
        #self.agents.remove(self.agent_name)
        
        # initializing request that needs to be processed
        self.current_request = {}
        
        # initializing replies dictionary
        self.replies = {}
        
        # variable used to count the number of replies recieved
        self.recieved_replies = 0
        
    
    
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

        
        
    
    def setup_pub_sub(self):
        # initializing orignial pub/sub
        super().setup_pub_sub()
        self.qos_profile_reply = rclpy.qos.QoSProfile(depth=len(self.agents), history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # creating a request publisher
        self.request_pub = self.create_publisher(SynchroRequest, '/synchro_request', self.qos_profile)
        
        # creating a request subscirber
        self.request_sub = self.create_subscription(SynchroRequest, '/synchro_request', self.request_callback, self.qos_profile)
        
        # creating publishers to reply to other agents and adding them to a dictionary
        self.reply_publishers = {}
        for agent in self.agents:
            self.reply_publishers[agent]= self.create_publisher(SynchroReply, agent+'/synchro_reply', self.qos_profile)
        
        self.reply_sub = self.create_subscription(SynchroReply, 'synchro_reply', self.reply_callback, self.qos_profile_reply)
           
        # creating a confirm publisher
        self.confirm_pub = self.create_publisher(SynchroConfirm, '/synchro_confirm', self.qos_profile)
        
        # creating a confirm subscirber
        self.confirm_sub = self.create_subscription(SynchroConfirm, '/synchro_confirm', self.confirm_callback, self.qos_profile)
        
        
        
        
    
    # returns just the first region for simplicity for now
    def chose_ROI(self, dependency, *args, **kwargs):
        return dependency[1][0]  
        

    def set_initial_state(self):        
        # if None then the initial state will be taken direcly from the dictionaries
        if not self.initial_state_ts_dict == None:
            # otherwise we modify the dictionaries to the state given 
            self.motion_dict['initial'] =  self.initial_state_ts_dict[self.motion_dict['type']]
            self.action_dict['initial'] =  self.initial_state_ts_dict[self.action_dict['type']]


    def timer_callback(self):
        request=self.ltl_planner.cooperative_action_in_horizon(self.starting_time, self.chose_ROI)
        self.starting_time += 1
        if request:
            self.get_logger().info('Synchro Planner: Sending Collaboration Request')
            self.current_request = request
            request_msg = self.build_request_msg(request)
            self.request_pub.publish(request_msg)

    
    def build_request_msg(self, request):
        request_msg = SynchroRequest()
        # adding name identifier
        request_msg.agent = self.agent_name
        # formatting the request
        for key, value in request.items():
            request_msg.roi.append(key[0])
            request_msg.actions.append(key[1])
            request_msg.time.append(value)
        return request_msg

    def request_callback(self, msg):
        #FIXMED: add check to discard if i'm the sender, and multiple request case
        agent, request = self.unpack_request_msg(msg)
        self.get_logger().info('Synchro Planner: Recieved Collaboration Request Form Agent: %s' %agent)     
        reply = self.ltl_planner.evaluate_request(request)
        reply_msg = self.build_reply_msg(reply)
        self.reply_publishers[agent].publish(reply_msg)
    
    def unpack_request_msg(self, msg):
        # getting name of requesting agent
        agent = msg.agent
        # building the request dictionary
        request = {}
        for i in range(len(msg.roi)):
            request[(msg.roi[i], msg.actions[i])] = msg.time[i]
        return agent, request
    
    def build_reply_msg(self, reply):
        reply_msg = SynchroReply()
        # building the reply message 
        reply_msg.agent = self.agent_name
        for key, value in reply.items():
            reply_msg.roi.append(key[0])
            reply_msg.actions.append(key[1])
            reply_msg.available.append(value[0])
            reply_msg.time.append(value[1])
        return reply_msg
        
    def reply_callback(self, msg):               
        # unpacking the reply message
        agent, reply = self.unpack_reply_msg(msg)        
        self.get_logger().info('Synchro Planner: Recieved Reply Form Agent: %s' %agent)     
        # updating the number of replies recievd
        self.recieved_replies += 1
        # if the agent can provide any help then we store the reply
        # this reduces the MIP computation time
        for result in reply.values():
            if result[0]:
                self.replies[agent] = reply
                break
        
        # TESTING
        '''
        reply1 = reply.copy()
        reply1[('r1','h1')] = (True, 50)        
        self.recieved_replies += 1
        for result in reply1.values():
            if result[0]:
                self.replies['/agent_2'] = reply1
                break
        
        reply2 = reply.copy()
        reply2[('r2','a12')] = (True, 20)
        self.recieved_replies += 1
        for result in reply2.values():
            if result[0]:
                self.replies['/agent_3'] = reply2
                break
        '''
        # END TESTING
                
        if self.recieved_replies == len(self.agents):     
            confirm, time = self.ltl_planner.confirmation(self.current_request, self.replies)
            self.replies = {}
            self.recieved_replies = 0               
            confirm_msg = self.build_confirm_msg(confirm, time)
            self.confirm_pub.publish(confirm_msg)    
    
    def unpack_reply_msg(self, msg):
        # getting name of requesting agent
        agent = msg.agent
        # building the request dictionary
        reply = {}
        for i in range(len(msg.roi)):
            reply[(msg.roi[i], msg.actions[i])] = (msg.available[i], msg.time[i])
        return agent, reply
    
    def build_confirm_msg(self, confirm, time):
        confirm_msg = SynchroConfirm()
        # adding the master agent
        confirm_msg.master = self.agent_name
        # check if a confirmation is possible
        if confirm == None:
            # setting time to a negative value to indicate that the action is not possible
            confirm_msg.time = -1.0
        else:
            # adding the time to execute the action
            confirm_msg.time = time        
            for agent, result in confirm.items():
                # checking if agent has to help, if so adding the action to the message
                for key, value in result.items():
                    if bool(value[0]):
                        confirm_msg.roi.append(key[0])
                        confirm_msg.actions.append(key[1])
                        confirm_msg.agents.append(agent)
                        break
        return confirm_msg
    
    def confirm_callback(self, msg):
        master, time, confirm = self.unpack_confirm_msg(msg)
        self.get_logger().info('Synchro Planner: Recieved Confirmation Form Agent: %s' %master)     
        
        # check if the action is possible
        if time == -1.0:
            if master == self.agent_name:
                print('I am the master')
                #TODOD: delay collaboration by a certain amount of time
            else:
                print('I am not involved')
                #TODOD: reset flags and all the other stuff   
        else:
            if master == self.agent_name:
                print('I am the master')
                #TODOD: adapt plan to meet the time if necessary
                
                # if wait in actions then no need to delaly               
            elif self.agent_name in confirm:
                print('I need to help')
                #TODOD: adapt detour to meet the time if necessary and add detour to plan
                
                # if wait in actions then no need to delaly
            else:
                print('I am not involved')    
                #TODOD: reset flags and all the other stuff 
        
        #TODOD: if queue start request again
        
        
        
    def unpack_confirm_msg(self, msg):
        confirm = {}
        for i in range(len(msg.agents)):
            agent = msg.agents[i]
            confirm[agent] = (msg.roi[i], msg.actions[i])
        return msg.master, msg.time, confirm    
        
        
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













        
        #print(self.ltl_planner.action_dictionary)
        #self.ltl_planner.index = 2
        #request = self.ltl_planner.cooperative_action_in_horizon(30, self.chose_ROI)

        
        #print('request: %s' % request)
        #print('run_line: %s' % self.ltl_planner.run.line)
        #print('run_loop: %s' % self.ltl_planner.run.loop)
        #print ('run_pre_plan: %s' % self.ltl_planner.run.pre_plan)
        #print ('run_pre_plan_cost: %s' % self.ltl_planner.run.pre_plan_cost)
        
        #self.ltl_planner.index = 0
        
        #reply1 = self.ltl_planner.evaluate_request(request)
        #print('reply: %s' % reply1) 
        #print('paths: %s' % self.ltl_planner.path)

        #reply2 = reply1.copy()
        #reply2[('r1','h1')] = (True, 50)
        #reply2[('r2','a2')] = (True, 20)

        #reply_dict={'/agent1': reply1, '/agent2': reply2, '/agent3': reply1}
        
        
        #confirm, time = self.ltl_planner.confirmation(request, reply_dict)
        #print('confirm: %s' % confirm)
        #print('time: %s' % time)
        #print(self.get_namespace())
        #print(confirm[self.get_namespace()])
        
        
        #self.ltl_planner.adapt_plan(('r2', 'a2'), time) 
        #print(self.ltl_planner.detour_path)
        #print (self.ltl_planner.detour)
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
        
        
        
        
        
        
        
        #for item in self.ltl_planner.product.edges(data=True):
            #print(item)
        print(self.ltl_planner.product.graph['initial'])    

        #show_automaton(self.ltl_planner.product, 'action')
        print(self.ltl_planner.curr_ts_state)
        print (self.ltl_planner.product.get_possible_states(('r3' , 'free')))
        self.ltl_planner.update_possible_states(('r3' , 'free'))
        print(self.ltl_planner.product.possible_states)
        self.ltl_planner.update_possible_states(('r2' , 'free'))
        print(self.ltl_planner.product.possible_states)
        self.ltl_planner.update_possible_states(('r2' , 'l123'))
        print(self.ltl_planner.product.possible_states)
        print(self.ltl_planner.product[((('r2', 'free'), 'T0_S9'))][(('r2', 'l123'), 'T0_S9')]['action'])
        print(self.ltl_planner.product[((('r2', 'free'), 'T0_S9'))][(('r2', 'l123'), 'T1_S7')]['action'])
 
        
        
        
        
        
        
    '''  
