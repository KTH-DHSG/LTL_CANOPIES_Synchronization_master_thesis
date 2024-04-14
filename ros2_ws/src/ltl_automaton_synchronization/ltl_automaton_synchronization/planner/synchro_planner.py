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
        super().__init__(ros_node, ts, hard_spec, soft_spec, beta, gamma)

    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
        # cooperative actions
        #FIXMED: PROBABLY I CAN REMOVE , *args, **kwargs BUT IT REMAINS TO BE CHECKED
        #TODOD: add self.cooperation_time_end
    def cooperative_action_in_horizon(self, horizon, chose_ROI, *args, **kwargs):
        k = 0
        j = self.index
        segment = self.segment
        request = {}
        # if i'm already in a contract
        if self.contract_time>0:
            return None
        while k<horizon:
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
                            roi = chose_ROI(item, *args, **kwargs)
                            # insert the request
                            request[(roi, item[0])] = k
                        # update contract time
                        self.contract_time = k
                        return request
                    # add weight of the last non collaborative action
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
                            roi = chose_ROI(item, *args, **kwargs)
                            # insert the request
                            request[(roi, item[0])] = k
                        # update contract time
                        self.contract_time = k
                        return request
                    # add weight of the last non collaborative action
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
                # IMPORTANTD:return time 0 instead of infinite
                reply[t_ts_node] = (False, 0)
            else:
                #TODOD: probably there is a better way to detour avoiding coming back exactly at the beginning but later
                # getting the initial state
                if self.segment == 'line':
                    f_ts_node = self.run.line[self.index]
                else:
                    f_ts_node = self.run.loop[self.index]
                
                # TODOD: add the minimization problem to fin the best position to place the detour
                #FIXMED: there is a problem in shortest path that if we are in a action we execute the action
                # again because want to go back to the initial state
                path[t_ts_node], time_ready, detour_time = self.shortest_path_ts(ts, f_ts_node, t_ts_node)
                # if a path exists
                if path[t_ts_node]:
                    # FIXMED: done to do the same thing as Meng's code i would use abs(time_ready-time)
                    reply[t_ts_node] = (True, time_ready + alpha*abs(time_ready-time))
                else:
                    reply[t_ts_node] = (False, 0)
        # save the path dictionary
        self.path = path.copy()
        return reply
        
        
        
    def confirmation(self, request, Reply):
        # show the layout
        Confirm, time = self.mip(request, Reply)
        if time:
            self.contract_time = time 
        return Confirm, time   
    
    
    # if called it add the detour to the plan
    #TODOD: modify find_next_move 
    def adapt_plan(self, t_ts_node, time):
        self.detour_path = self.path[t_ts_node]
        for i in range(len(self.detour_path)-1):
            action = self.product.graph['ts'][self.detour_path[i]][self.detour_path[i+1]]['action']
            self.detour.append(action)
        # empty the path    
        #self.path = {}
        self.next_move = self.detour[0]
        self.dindex = 0
        self.contract_time = time
        self.segment = 'detour'
        return True
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #===========================================
    # shortest path in ts
    #===========================================

    def shortest_path_ts(self, ts, f_ts_node, t_ts_node):
        # if the nodes are not in the ts
        if (f_ts_node not in ts) or (t_ts_node not in ts):
            return None, None, None
        else:
            # compute path to target node
            path_pre, path_dist = dijkstra_predecessor_and_distance(ts, f_ts_node)
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
            total_cost= path_dist_back[f_ts_node]+path_dist[t_ts_node]
        return (final_path, time_ready, total_cost)

    
    
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
            self.ros_node.get_logger().warning('GUROBI: No solution found!')
            return None, None

    
    