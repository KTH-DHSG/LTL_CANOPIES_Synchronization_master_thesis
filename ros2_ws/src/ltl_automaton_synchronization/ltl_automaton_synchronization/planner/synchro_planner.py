import os
import sys
import rclpy
from ltl_automaton_planner.ltl_tools.ltl_planner import LTLPlanner
from ltl_automaton_planner.ltl_tools.discrete_plan import compute_path_from_pre, dijkstra_targets
from networkx import dijkstra_predecessor_and_distance

from gurobipy import *
from collections import defaultdict

class sync_LTLPlanner(LTLPlanner):
    def __init__(self, ros_node, ts, hard_spec, soft_spec, beta=1000, gamma=10):
        self.action_dictionary = ts.graph['action'].action
        self.contract_time = 0
        self.path = {}
        self.detour_path = []
        self.detour = []
        self.new_segment = ''
        self.past_segment = ''
        self.new_index = 0
        super().__init__(ros_node, ts, hard_spec, soft_spec, beta, gamma)
        self.past_conditions = {} # t_ts_node: (past_segment, new_segment, new_index) conditions which determines where i come back
        # variables used to take into account time required to finish current action
        self.last_weight = 0
        self.start_last_move = 0
        # cooperative actions
        
    def cooperative_action_in_horizon(self, horizon, prev_received_timestamp, chose_ROI):
        actual_time = self.ros_node.get_clock().now().to_msg()
        last_action_elapsed_time = round(((actual_time.sec + actual_time.nanosec/1e9)- (prev_received_timestamp.sec + prev_received_timestamp.nanosec/1e9)), 2)
        first_action_flag = True
        k = 0
        j = self.index
        # adapting index in case of detour
        if self.segment =='detour':
            j=self.dindex
        segment = self.segment
        request = {}
        # if i'm already in a contract
        if self.contract_time>0:
            return None
        while k<horizon:
            #checking also in the detour if if i have one
            #note that if contract_time is 0 in the detour i cannot have collaborative actions
            # but i need to just account for the actions that are not collaborative
            if segment == 'detour':
                if j<len(self.detour):
                    act_key = self.key_given_label(self.detour[j])
                    # add weight of the last non collaborative action
                    if first_action_flag:
                        k += max(0, self.action_dictionary[act_key]['weight']-last_action_elapsed_time)
                        first_action_flag = False
                    else:
                        k += self.action_dictionary[act_key]['weight']
                    j += 1
                else:
                    # update index and segment
                    j=self.index
                    segment = self.new_segment    
            # in the prefix plan
            if segment == 'line':
                # check if i'm still in the prefix plan
                if j<len(self.run.pre_plan):
                    # get the key of the action
                    act_key = self.key_given_label(self.run.pre_plan[j])
                    # if the action is collaborative
                    if self.action_dictionary[act_key]['type']== 'collaborative':
                        #insert the dependency in the request
                        for item in self.action_dictionary[act_key]['dependency'].items():
                            # chose the ROI
                            roi = chose_ROI(item)
                            # insert the request
                            request[(roi, item[0])] = k
                        # update contract time
                        self.contract_time = k
                        return request
                    # add weight of the last non collaborative action
                    if first_action_flag:
                        k += max(0, self.action_dictionary[act_key]['weight']-last_action_elapsed_time)
                        first_action_flag = False
                    else:
                        k += self.action_dictionary[act_key]['weight']
                    j += 1
                # otherwise switch to the suffix plan
                else:
                    j=0
                    segment ='loop'            
            if segment =='loop':
                # if we do not need to loop back to the beginning of the suffix plan
                if j<len(self.run.suf_plan):
                    # get the key of the action
                    act_key = self.key_given_label(self.run.suf_plan[j])
                    # if the action is collaborative
                    if self.action_dictionary[act_key]['type'] == 'collaborative':
                        #insert the dependency in the request
                        for item in self.action_dictionary[act_key]['dependency'].items():
                            # chose the ROI
                            roi = chose_ROI(item)
                            # insert the request
                            request[(roi, item[0])] = k
                        # update contract time
                        self.contract_time = k
                        return request
                    # add weight of the last non collaborative action
                    if first_action_flag:
                        k += max(0, self.action_dictionary[act_key]['weight']-last_action_elapsed_time)
                        first_action_flag = False
                    else:
                        k += self.action_dictionary[act_key]['weight']
                    j += 1
                # if we need to loop back to the beginning of the suffix plan
                else:
                    j=0
        # if we are at the end of the horizon and no collaborative actions has been found
        return None
    
    def evaluate_request(self, request, alpha=1):
        reply = dict()
        path = dict()
        for t_ts_node, time in request.items():
            ts = self.product.graph['ts']
            # if i'm already in a contract or the node is not in the ts
            if (self.contract_time)>0 or (not ts.has_node(t_ts_node)):
                reply[t_ts_node] = (False, 0)
            else:
                now= self.ros_node.get_clock().now().to_msg()
                now = now.sec + now.nanosec/1e9
                #calculating time remaining for current action
                time_remaining = max(0, self.last_weight-(now- self.start_last_move))
               # getting the initial state
                #print("segment: ", self.segment)
                #print("index: ", self.index)
                #print("line: ", self.run.line)
                #print("loop: ", self.run.loop)          
                
                
                #if my curren action will bring me in a state inside the line plan
                if self.segment == 'line' and self.index < len(self.run.line)-1:
                    s_ts_node = self.run.line[self.index+1]
                    # if the next action is the last of the line plan
                    if self.index < len(self.run.line)-2:
                        f_ts_node = self.run.line[self.index+2]
                        self.past_conditions[t_ts_node] = ('line', 'line', self.index+1)
                    # else it will be the first one in the loop
                    else:
                        f_ts_node = self.run.loop[0]
                        self.past_conditions[t_ts_node] = ('line', 'loop', 0) 
                # if the current anction will bring me at the end of the line plan
                elif self.segment == 'line' and self.index == len(self.run.line)-1:
                    s_ts_node = self.run.loop[0]
                    f_ts_node = self.run.loop[1]
                    self.past_conditions[t_ts_node] = ('line', 'loop', 0)     
                elif self.segment == 'loop' and self.index < len(self.run.loop)-1:
                    s_ts_node = self.run.loop[self.index+1]
                    # if the next action is the last of the loop plan
                    if self.index < len(self.run.loop)-2:
                        f_ts_node = self.run.loop[self.index+2]
                        self.past_conditions[t_ts_node] = ('loop', 'loop', self.index+1)
                    # else it will be the first one in the loop
                    else:
                        f_ts_node = self.run.loop[0]
                        self.past_conditions[t_ts_node] = ('loop', 'loop', 0)
                elif self.segment == 'loop' and self.index == len(self.run.loop)-1:
                    s_ts_node = self.run.loop[0]
                    f_ts_node = self.run.loop[1]
                    self.past_conditions[t_ts_node] = ('loop', 'loop', 0)                                
                print("past_conditions: ", self.past_conditions[t_ts_node])
                # again because want to go back to the initial state
                path[t_ts_node], time_ready = self.shortest_path_ts(ts,s_ts_node, f_ts_node, t_ts_node)
                #adding also the time remaining for the current action
                reply[t_ts_node] = (True, time_ready+time_remaining)
        # save the path dictionary
        self.path = path.copy()
        return reply
        
        
        
    def confirmation(self, request, Reply):
        # compute the MIP
        Confirm, time = self.mip(request, Reply)
        # update contract time
        if time:
            self.contract_time = time 
        return Confirm, time   
    
    
    # if called it add the detour to the plan
    def adapt_plan(self, t_ts_node, time):
        self.detour_path = self.path[t_ts_node]
        # empty the detour list
        self.detour = []
        for i in range(len(self.detour_path)-1):
            action = self.product.graph['ts'][self.detour_path[i]][self.detour_path[i+1]]['action']
            self.detour.append(action)
        # empty the path
        self.path = {}
        # saving the current updating index
        self.dindex = 0
        self.contract_time = time
        # saving past segment type to know where to start again
        self.past_segment = self.past_conditions[t_ts_node][0]
        self.new_segment = self.past_conditions[t_ts_node][1]
        self.new_index = self.past_conditions[t_ts_node][2]
        # empty past conditions
        self.past_conditions = {}
        self.segment = 'detour'
        self.ros_node.get_logger().warning('Adapting plan to detour: '+str(self.detour))
    
    
    def delay_collaboration(self, time):
        detour_length = round(time/self.action_dictionary['free']['weight'])
        # creating a detour of none actions
        self.detour = ['none']*detour_length
        
        if self.segment == 'line':
            self.past_segment = 'line'
            # checking the TS state after the current action
            if self.run.line[self.index][1] =='free':
                # we are not in an acrion state
                new_state= self.run.line[self.index]
            else:
                # we are in an action state
                new_state= (self.run.line[self.index][0], 'free')
            # computing the detour path
            self.detour_path = [new_state]*(detour_length+1)
            # updating segment index
            self.dindex = 0
        elif self.segment == 'loop':
            self.past_segment = 'loop'
            # checking the TS state after the current action
            if self.run.loop[self.index][0] =='free':
                # we are not in an acrion state
                new_state= self.run.loop[self.index]
            else:
                # we are in an action state
                new_state = (self.run.loop[self.index][0], 'free')
            # computing the detour path
            self.detour_path = [new_state]*(detour_length+1)
            # updating segment index
            self.dindex = 0
        # we are already in a detour
        else:
            # computing the detour path making it one step longer since we will start from index 1
            self.detour_path = [self.curr_ts_state]*(detour_length+2)
            self.detour.append('none')
            # updating segment index set like this for compatibility reasons with find_next_move
            self.dindex = 1
        # updating the segment
        self.segment = 'detour'
        # removing contract time to allow checks in horizon
        self.contract_time = 0
    
    def find_next_move(self):
        self.start_last_move = self.ros_node.get_clock().now().to_msg()
        self.start_last_move = self.start_last_move.sec + self.start_last_move.nanosec/1e9
        # I have a detour to complete
        if self.segment == 'detour':
            # I'm starting a detour 
            if self.dindex == 0:
                if self.past_segment == 'line':
                    # Add the node that has been visited to trace
                    self.trace.append(self.run.line[self.index])   
                elif self.past_segment == 'loop':
                    # Add the node that has been visited to trace
                    self.trace.append(self.run.loop[self.index])
                
                # update the index
                self.index= self.new_index
                # Extract next move from detour
                self.next_move = self.detour[self.dindex]
                # Increment index counter
                self.dindex += 1
            # i'm in the middle of a detour
            elif self.dindex < len(self.detour)-1:            
                # Add the node that has been visited to trace
                self.trace.append(self.detour_path[self.dindex+1])                
                # Extract next move from pre_plan
                self.next_move = self.detour[self.dindex]
                # Increment index counter
                self.dindex += 1                
            # i'm finishing a detour
            elif self.dindex == len(self.detour)-1:
                # Add the node that has been visited to trace
                self.trace.append(self.detour_path[self.dindex+1])                
                # Extract next move from pre_plan
                self.next_move = self.detour[self.dindex]
                # Increment index counter
                self.dindex += 1             
                # updating the segment
                self.segment = self.new_segment
                # free agent at the end of the detour
                self.contract_time = 0
            # returning the next move
            return self.next_move
        else:
            # Use original function
            return super().find_next_move()

       
    
    
    #===========================================
    # shortest path in ts
    #===========================================

    def shortest_path_ts(self, ts, s_ts_node, f_ts_node, t_ts_node):
        # start_ts_node , final_ts_node, target_ts_node
        # if the nodes are not in the ts
        if (f_ts_node not in ts) or (t_ts_node not in ts) or (s_ts_node not in ts):
            return None, None, None
        else:
            # compute path to target node
            path_pre, path_dist = dijkstra_predecessor_and_distance(ts, s_ts_node)
            # compute path from target node
            path_pre_back, path_dist_back = dijkstra_predecessor_and_distance(ts, t_ts_node)
            # if a loop  starting and finishing in the initial state and passing from the target state does not exist
            if (t_ts_node not in path_dist) or (f_ts_node not in path_dist_back):
                return None, None, None  
            # compute path from the initial state to the target state        
            path = compute_path_from_pre(path_pre, t_ts_node)
            # compute path from the target state to the initial state
            path_back = compute_path_from_pre(path_pre_back, f_ts_node)
            # combine the path into a single one
            final_path = []
            for i in range (0, len(path)-1):
                final_path.append(path[i])
            for i in range (0, len(path_back)):
                final_path.append(path_back[i])
            # compute the time ready to reach the target state
            time_ready = path_dist[path[len(path)-2]]
            # compute the total cost to of the detour
            print('Path:', final_path)
        return (final_path, time_ready)

    
    
    def key_given_label(self, label):
        for key, value in self.action_dictionary.items():
            if value['label'] == label:
                return key
        return None
    
    
    
    
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

    
    