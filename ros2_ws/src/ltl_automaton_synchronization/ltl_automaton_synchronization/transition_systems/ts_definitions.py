from networkx import DiGraph
from math import sqrt
from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard


# function to calculate 2D distance between two poses
def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

#===================================================
#           Motion Transition System
#===================================================
class MotionTS(DiGraph):
    
    # node_dict = { symbol : {label, pose, weights}}    
    def __init__(self, node_dict):
        DiGraph.__init__(self, symbols=set(node_dict['regions'].keys()), node_dict=node_dict['regions'], ts_type=node_dict['type'], initial=node_dict['initial'])
        # build the full graph
        self.build_full()
        # set the initial node of the graph
        #self.set_initial(init_pose)
        
    def build_full(self):
        #creating only nodes
        self.build_nodes()
        # creating all the edges
        self.build_edges()
            
    def build_nodes(self):                
        for (n, attrib) in self.graph['node_dict'].items():
            self.add_node(n, label=attrib['label'], pose=attrib['pose'], weights=attrib['weights'] )

    # add edges to the graph
    def build_edges(self):
        for (i_node, i_data) in self.nodes(data=True):
            j=0
            for j_node in self.nodes():
                if (i_node, j_node) not in self.edges():
                    if(i_node != j_node):
                        self.add_edge(i_node, j_node, weight = i_data['weights'][j])
                j+=1

    # find the closest node to the given pose if it is in the graph
    def closest_node(self, pose):        
        # check if the pose is in the graph       
        possible_nodes = []
        for (node, data) in self.nodes(data=True):
            node_pose = data['pose']
            if distance(node_pose,pose) <= node_pose[2]:
                possible_nodes.append(node)
        # if the pose is not in the graph, raise an exception
        if len(possible_nodes) == 0:
            return None        
        # if the pose is in the graph, return the closest node        
        node = min(possible_nodes, key= lambda n: distance(self.nodes[n]['pose'],pose))       
        return node

#===================================================
#                   Action Model
#===================================================
class ActionModel(object):    
    # action_dict = {act_name: (cost, guard_formula, label, dependendy={action: set(regions)}, category)}
    def __init__(self, action_dict):
        self.action = action_dict['actions']
        self.ts_type = action_dict['type']
        self.initial = action_dict['initial']
        
    # return the allowed actions for a given node label i.e. region of the motion FTS
    # works with single labels or sets of labels independently
    def allowed_actions(self, ts_node_label):
        allow_action = set()
        for act_name, attrib in self.action.items():
            if (parse_guard(attrib["guard"]).check(ts_node_label)):
                allow_action.add(act_name)
        return allow_action
    
#===================================================
#     Complete Motion-Action Transition System       
#===================================================       
class MotActTS(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set([(mot_fts.graph['initial'], act_model.initial)]), ts_state_format=[[mot_fts.graph['ts_type']], [act_model.ts_type]])

    # creates the states for the complete model
    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            # modification to make action none as state free
            self.add_node(prod_node, label=prod_node, region=self.graph['region'].nodes[reg]['pose'], action=act, marker='unvisited')
        return prod_node 
    
    def build_full(self):
        #creating only valid nodes
        self.build_nodes()
        # creating all the edges
        self.build_edges()
        
        # creating initial node
                                    
    def build_nodes(self):
        for reg in self.graph['region'].nodes():
            for act in self.graph['action'].allowed_actions(reg):
                self.composition(reg, act)
        
    def build_edges(self):
        movement_actions = {}
        for reg in self.graph['region'].nodes():
            for act in self.graph['action'].action.keys():
                # adding actions transitions
                for act_to in self.graph['action'].allowed_actions(reg):
                    if act_to != 'free':
                        # create action node
                        if(act=='free'):
                            # add edge from None to action
                            #IMPORTANTD: chnaged label to action (just the name to adapt to the planner)
                            self.add_edge((reg, act), (reg, act_to), action = self.graph['action'].action[act_to]['label'], guard=self.graph['action'].action[act_to]['guard'], weight=self.graph['action'].action[act_to]['weight'], marker= 'visited')
                        # add edge from action to None
                        self.add_edge((reg, act_to), (reg, 'free'), action='none', guard='1', weight=self.graph['action'].action['free']['weight'], marker='visited')
                # adding motions transitions
                for reg_to in self.graph['region'].successors(reg):
                    # add edge between regions (actions None for both nodes)
                    act = 'goto_'+str(reg)+'_'+str(reg_to)
                    cost = self.graph['region'][reg][reg_to]['weight']
                    self.add_edge((reg, 'free'), (reg_to, 'free'), action=act, guard='1', weight=cost, marker='visited')
                    # adding all movement actions to a dictionary
                    movement_actions.update({act: {'weight':cost, 'guard':'1', 'label': act , "dependency": {}, 'type': 'local'}})
            # adding self transitions (none action to waste time)
            self.add_edge((reg, 'free'), (reg, 'free'), action='none', guard='1', weight=self.graph['action'].action['free']['weight'], marker='visited')
        self.graph['action'].action.update(movement_actions)    
    
    def set_initial(self, ts_state):
        """
        Delete and set new initial state.
        """
        # If state exist in graph, change initial and return true
        if ts_state in self.nodes():
            self.graph['initial'] = set([ts_state])
            return True
        # If state doesn't exist in graph, return false
        else:
            return False
