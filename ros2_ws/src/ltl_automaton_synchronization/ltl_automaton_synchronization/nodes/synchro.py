import rclpy
from rclpy.executors import MultiThreadedExecutor

from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner
from ltl_automaton_planner.nodes.planner_node import MainPlanner
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
from ltl_automaton_synchronization.planner.synchro_planner import sync_LTLPlanner
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ltl_automaton_msg_srv.srv import FinishCollab
from ltl_automaton_msg_srv.msg import SynchroConfirm, SynchroRequest, SynchroReply

from std_msgs.msg import String
import networkx as nx
import matplotlib.pyplot as plt
import yaml
from random import shuffle

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
        self.agents = self.declare_parameter('agents', ['']).value 
        
        # getting all the agents involved
        self.time_horizon = self.declare_parameter('time_horizon', 10).value 

        
        # initializing request that needs to be processed
        self.current_request = {}
        
        # initializing the request queue
        self.request_queue = []
        
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
        nodes = len(self.robot_model.nodes())
        self.get_logger().warn('Nodes wFTS: %d' % nodes)
        
        
    
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
        
        #creating service to update planner after a collaboration
        finished_collab_srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.finished_collab_srv = self.create_service(FinishCollab, 'finished_collab', self.finished_collab_callback, callback_group=finished_collab_srv_cb_group)
        
        #used to update last actions for the planner
        # Initialize publisher to send plan commands
        self.plan_sub = self.create_subscription(String, 'next_move_cmd', self.next_move_cmd_callback, self.qos_profile)
        
        
    #IMPORTANTD: this function must be adapted for each simulation
    # returns just the first region for simplicity for now
    
    # for thesis
    ''' 
     def chose_ROI(self, dependency):
        if self.agent_name == '/turtlebot1':
            return dependency[1][1]
        if self.agent_name == '/turtlebot2':
            return dependency[1][0]
        
        return dependency[1][0]   
    '''
  
    # for paper
    def chose_ROI(self, dependency):
        if self.agent_name == '/turtlebot5':
            return dependency[1][1]
        if self.agent_name == '/turtlebot7':
            return dependency[1][0]
        
        return dependency[1][0]   

    def set_initial_state(self):        
        # if None then the initial state will be taken direcly from the dictionaries
        if not self.initial_state_ts_dict == None:
            # otherwise we modify the dictionaries to the state given 
            self.motion_dict['initial'] =  self.initial_state_ts_dict[self.motion_dict['type']]
            self.action_dict['initial'] =  self.initial_state_ts_dict[self.action_dict['type']]


    def timer_callback(self):
        request=self.ltl_planner.cooperative_action_in_horizon(self.time_horizon, self.prev_received_timestamp, self.chose_ROI)
        if request:
            self.get_logger().info('Synchro Planner: Sending Collaboration Request')
            #self.current_request = request
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
        agent, request = self.unpack_request_msg(msg)
        self.get_logger().info('Synchro Planner: Recieved Collaboration Request Form Agent: %s' %agent)     
        # I'm not processing another request
        if not self.current_request: 
            self.get_logger().info('Synchro Planner: Processing Collaboration Request Form Agent: %s' %agent)    
            message= 'agent '+str(agent)
            self.get_logger().info(f"\033[34m{message}\033[0m")
            self.current_request = request            
            reply = self.ltl_planner.evaluate_request(request)
            reply_msg = self.build_reply_msg(reply)
            self.reply_publishers[agent].publish(reply_msg)
        else:
            # if I'm processing another request then I add the request to the queue
            self.request_queue.append((agent, request))
    
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
        # once all the replies are recieved we can confirm the collaboration
        if self.recieved_replies == len(self.agents):
            # filtering replies to reduce the number of agents involved in the MIP
            t_m = list(self.current_request.values())[0]
            m = len(self.current_request)
            # if a solution exists then we filter the replies and compute the reduced MIP
            if len(self.replies) >= m:            
                filtered_replies = self.filter_replies(self.replies, t_m, m) 
                # getting the confirmation
                confirm, time = self.ltl_planner.confirmation(self.current_request, filtered_replies)
            # if no solution exists then we return None
            else:
                confirm, time = None, None
            # empty the replies dictionary
            self.replies = {}
            # reset the number of replies recieved
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
    
    def filter_replies(self, replies, t_m, m):
        useful_agents = []
        formatted_replies = {}
        filtered_replies = {}
        for agent, reply in replies.items():
            for key, value in reply.items():
                # during the first iteration i create the values as a list
                if key not in formatted_replies:
                    formatted_replies[key] = []
                # if an agent can help compute the metrics
                if value[0]:
                    formatted_replies[key].append([agent, abs(value[1]-t_m)])
                # otherwise the value of the metric is infinity
                else:
                    formatted_replies[key].append([agent, float('inf')])
        for key, value in formatted_replies.items():
            # shuffling the list to guarantee that the agents are selected randomly
            shuffle(value)
            # sorting in ascending order of time
            value.sort(key=lambda x: x[1])
            # adding the first M agents for the specific action
            for i in range(m):
                # adding them only once since i'm using a list
                if value[i][0] not in useful_agents:
                    useful_agents.append(value[i][0])
        # filtering the replies
        for agent in useful_agents:
            filtered_replies[agent] = replies[agent]
        # returning the filtered replies
        return filtered_replies    

    
    def confirm_callback(self, msg):
        master, time, confirm = self.unpack_confirm_msg(msg)
        self.get_logger().info('Synchro Planner: Recieved Confirmation Form Agent: %s' %master) 
        # if no solution exists
        if time == -1.0:
            # I'm the agent who sent the request
            if master == self.agent_name:
                self.get_logger().warning('I need to delay collaboration')
                # we delay the collaboration by a time horizon
                self.ltl_planner.delay_collaboration(self.time_horizon)
            # I'm an agent who answered the request
            else:
                self.get_logger().warning('I am not involved')
                # empty the paths created while processing the request
                self.ltl_planner.path = {}         
        # if a solution exists
        else:
            # I'm the agent who sent the request
            if master == self.agent_name:
                self.get_logger().warning('I am the master')
                # update contract time
                self.ltl_planner.contract_time = time
            elif self.agent_name in confirm:
                self.get_logger().warning('I need to help')
                # adapting the plan
                self.ltl_planner.adapt_plan(confirm[self.agent_name], time)
            else:
                self.get_logger().warning('I am not involved')    
                # empty the paths created while processing the request
                self.ltl_planner.path = {}         
        # if there are requests in the queue then we process them
        if self.request_queue:
            # get the first request in the queue
            agent, request = self.request_queue.pop(0)
            self.get_logger().info('Synchro Planner: Processing Collaboration Request Form Agent: %s' %agent)    
            self.current_request = request            
            reply = self.ltl_planner.evaluate_request(request)
            reply_msg = self.build_reply_msg(reply)
            self.reply_publishers[agent].publish(reply_msg)
        else:
            # empty the current request
            self.current_request = {}
        
            
    def unpack_confirm_msg(self, msg):
        confirm = {}
        for i in range(len(msg.agents)):
            agent = msg.agents[i]
            confirm[agent] = (msg.roi[i], msg.actions[i])
        return msg.master, msg.time, confirm   
    
    def finished_collab_callback(self, request, response):
        self.get_logger().info('Synchro Planner: Collaboration Finished')
        # updates the planner after a collaboration
        self.ltl_planner.contract_time = 0
        return response
     
    def next_move_cmd_callback(self, msg):
        now = self.get_clock().now().to_msg()
        self.ltl_planner.start_last_move = now.sec+now.nanosec*1e-9
        
        enty =''
        for key, value in self.robot_model.graph['action'].action.items():
            if value['label'] == msg.data:
                entry = key
                break
        self.ltl_planner.last_weight = self.robot_model.graph['action'].action[entry]['weight']
    #-----------------------------------------------------
    # Check if given TS state is the next one in the plan
    #-----------------------------------------------------
    def is_next_state_in_plan(self, ts_state):
        # check if the next state is in the detour path
        if self.ltl_planner.segment == 'detour':
            if ts_state == self.ltl_planner.detour_path[self.ltl_planner.dindex]:
                return True
        else:
            # calls the origina function
            return super().is_next_state_in_plan(ts_state)






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

