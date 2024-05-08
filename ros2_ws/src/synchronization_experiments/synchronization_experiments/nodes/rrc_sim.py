import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ltl_automaton_msg_srv.msg import SynchroConfirm, SynchroRequest, SynchroReply
from std_msgs.msg import String
from random import shuffle, choice, uniform
from gurobipy import *
from collections import defaultdict

class RRC(Node):
    def __init__(self):
        super().__init__('rrc')
        self.init_params()
        self.setup_pub_sub()
        self.get_logger().info(self.get_namespace() + ' is ready')
        if self.get_namespace()=='/agent0':
            time=self.get_clock().now().to_msg().sec
            flag=True
            while(self.get_clock().now().to_msg().sec-time<self.waiting):
                if (self.get_clock().now().to_msg().sec-time)%10==0:
                    if flag:
                        self.get_logger().warn('Waiting')
                        flag=False
                else:
                    flag=True                    
                pass
            self.pub_request()
    def init_params(self):
        # number of agents for the simulation
        self.num_agents = self.declare_parameter('num_agents', 1).value
        
        # creating agents list
        self.agents =[]
        for i in range(self.num_agents):
            self.agents.append('/agent'+str(i))
        
        # updating the agent name
        self.agent_name = self.get_namespace()
        
        self.num_ass_actions = self.declare_parameter('num_ass_actions', 1).value
        
        # flag to activate filtering
        self.filter = self.declare_parameter('filter', True).value
        
        #time to wit before starting the request
        self.waiting= self.declare_parameter('waiting', 10).value
        # initializing request that needs to be processed
        self.current_request = {}
                
        # initializing replies dictionary
        self.replies = {}
        
        # variable used to count the number of replies recieved
        self.recieved_replies = 0
        
        #start and ending time for RRC cycle
        self.start_time = 0
        self.mid_time = 0
        self.end_time = 0 
        self.agents_involved = 0
    
    def setup_pub_sub(self):
        len(self.agents)
        self.qos_profile = rclpy.qos.QoSProfile(depth=1, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
       
        self.qos_profile_reply = rclpy.qos.QoSProfile(depth=len(self.agents), history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        # creating a request publisher
        self.request_pub = self.create_publisher(SynchroRequest, '/synchro_request', self.qos_profile)
        
        # creating a request subscirber
        self.request_sub = self.create_subscription(SynchroRequest, '/synchro_request', self.request_callback, self.qos_profile)
        
        # creating publishers to reply to other agents and adding them to a dictionary
        self.reply_publisher = self.create_publisher(SynchroReply, '/agent0/synchro_reply', self.qos_profile)
        
        if self.get_namespace()=='/agent0':
            cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
            self.reply_sub = self.create_subscription(SynchroReply, 'synchro_reply', self.reply_callback, self.qos_profile_reply, callback_group=cb_group)
           
        # creating a confirm publisher
        self.confirm_pub = self.create_publisher(SynchroConfirm, '/synchro_confirm', self.qos_profile)
        
        # creating a confirm subscirber
        #self.confirm_sub = self.create_subscription(SynchroConfirm, '/synchro_confirm', self.confirm_callback, self.qos_profile)
        

    def pub_request(self):
        request = {}
        for i in range(self.num_ass_actions):
            # request entry for each assisitve action at time 10
            request [('r1', 'a'+str(i))] = 10
        self.get_logger().info('Synchro Planner: Sending Collaboration Request')
        request_msg = self.build_request_msg(request)
        time_msg = self.get_clock().now().to_msg()
        self.start_time = time_msg.sec+time_msg.nanosec*1e-9        
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
        # get the agent
        agent, request = self.unpack_request_msg(msg)
        # update current request
        self.current_request = request 
        reply={}
        for t_ts_node, time in request.items():
            # random value to define if able or not to do the action
            b_d=choice([True, False])
            if b_d:   
                # random time to define when agent is able to help             
                reply[t_ts_node] = (b_d, uniform(5, 15))
            else:
                reply[t_ts_node] = (b_d, 0)
        waste_time=uniform(0, 6)      
        time=self.get_clock().now().to_msg()
        time=time.sec+time.nanosec*1e-9
        updated_time=self.get_clock().now().to_msg()
        updated_time=updated_time.sec+updated_time.nanosec*1e-9
        while(self.get_clock().now().to_msg().sec+self.get_clock().now().to_msg().nanosec-time<waste_time):          
            updated_time=self.get_clock().now().to_msg()
            updated_time=updated_time.sec+updated_time.nanosec*1e-9
        reply_msg = self.build_reply_msg(reply)
        self.reply_publisher.publish(reply_msg)

    
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
        # updating the number of replies recievd
        self.recieved_replies += 1
        print(self.recieved_replies)
        # if the agent can provide any help then we store the reply
        # this reduces the MIP computation time
        if self.filter:
            for result in reply.values():
                if result[0]:
                    self.replies[agent] = reply
                    break                
        else:
            self.replies[agent] = reply
        # once all the replies are recieved we can confirm the collaboration
        if self.recieved_replies == len(self.agents):
            self.mid_time = self.get_clock().now().to_msg().sec+self.get_clock().now().to_msg().nanosec*1e-9
            # filtering replies to reduce the number of agents involved in the MIP
            t_m = list(self.current_request.values())[0]
            m = len(self.current_request)
            # if a solution exists then we filter the replies and compute the reduced MIP
            if len(self.replies) >= m:
                if self.filter:            
                    filtered_replies = self.filter_replies(self.replies, t_m, m) 
                else:
                    filtered_replies = self.replies
                # getting the confirmation
                self.agents_involved = len(filtered_replies)
                confirm, time = self.mip(self.current_request, filtered_replies)
            # if no solution exists then we return None
            else:
                confirm, time = None, None
            # empty the replies dictionary
            self.replies = {}
            # reset the number of replies recieved
            self.recieved_replies = 0               
            confirm_msg = self.build_confirm_msg(confirm, time)
            time_msg = self.get_clock().now().to_msg()
            self.end_time = time_msg.sec+time_msg.nanosec*1e-9
            self.get_logger().warn("Agents in the MIP: "+str(self.agents_involved))
            self.get_logger().warn("RRC time: "+str(self.end_time-self.start_time))
            self.get_logger().warn("Filter and MIP time: "+str(self.end_time-self.mid_time))
            self.get_logger().info(str(confirm_msg))
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
            
    def unpack_confirm_msg(self, msg):
        confirm = {}
        for i in range(len(msg.agents)):
            agent = msg.agents[i]
            confirm[agent] = (msg.roi[i], msg.actions[i])
        return msg.master, msg.time, confirm  
    
    
    
     
    # Mixed Integer Programming (MIP) function for assignment problem
    def mip(self, request, Reply):
        # Get keys of request and Reply
        action_list = list(request.keys())
        agent_list = list(Reply.keys())
        
        # Check if there is no solution
        # no agent can perfrom a specific action
        for action in action_list:
            if all(Reply[a][action][0] == False for a in agent_list):
                self.ros_node.get_logger().warning('GUROBI: No solution found!')
                return None, None
        try:
            # Use Gurobi solver for MIP
            
            M = len(Reply.keys())   # Number of agents
            N = len(request.keys()) # Number of actions
            
            # initialize problem
            assign = defaultdict(lambda: [0,]*N)
            m = Model("assignment")
            # Set Gurobi parameters not to log to console
            m.Params.LogToConsole = 0
        
            # Create variables
            for i in range(0, M):
                for j in range(0, N):
                    assign[i][j] = m.addVar(vtype=GRB.BINARY, name="b[%s][%s]" % (str(i), str(j)))
            m.update()
            
            # Set objective
            obj = 0
            for i in range(0, M):
                for j in range(0, N):
                    obj += (assign[i][j] * Reply[agent_list[i]][action_list[j]][0] *
                            abs(Reply[agent_list[i]][action_list[j]][1] - request[action_list[j]]))
            m.setObjective(obj, GRB.MINIMIZE)
            
            # Add constraints
            # Each agent can only be assigned to one action at most
            for i in range(0, M):
                constr = 0
                for j in range(0, N):
                    constr += assign[i][j] * Reply[agent_list[i]][action_list[j]][0]
                m.addConstr(constr <= 1, 'row%s' % str(i))
            
            # Each action must be assigned to one agent
            for j in range(0, N):
                constr = 0
                for i in range(0, M):
                    constr += assign[i][j] * Reply[agent_list[i]][action_list[j]][0]
                m.addConstr(constr == 1, 'col%s' % str(j))
            
            # Solve
            m.optimize()
            
            # Check if optimal solution is found
            if m.status != GRB.OPTIMAL:
                self.ros_node.get_logger().warning('GUROBI: No solution found!')
                return None, None

            
            # Send confirmation
            time_list1 = [assign[i][j].x * Reply[agent_list[i]][action_list[j]][0] * Reply[agent_list[i]][action_list[j]][1]
                          for i in range(0, M) for j in range(0, N)]
            time_list2 = [request[action_list[j]] for j in range(0, N)]
            time = max(time_list1 + time_list2)
            Confirm = dict()
            # Create confirmation dictionary
            for i in range(0, M):
                confirm = dict()
                for j in range(0, N):
                    if assign[i][j].x:
                        confirm[action_list[j]] = (assign[i][j].x, time)
                    else:
                        confirm[action_list[j]]= (assign[i][j].x, 0)
                Confirm[agent_list[i]] = confirm
            
            return Confirm, time 
        
        except GurobiError:
            self.ros_node.get_logger().warning('GUROBI: There was an error!')
            return None, None

    
    
    








#==============================
#             Main
#==============================
def main():
    rclpy.init()   
    node = RRC() 
    executor = MultiThreadedExecutor() 
    executor.add_node(node)
    executor.spin()



if __name__ == '__main__':
    main()

