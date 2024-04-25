from motion_ts import MotionFts, MotionFts2
from action_mod import ActionModel
from networkx import DiGraph

class MotActModel(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set(), type='MotActModel')

    # creates the states for the complete model
    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            print(act)
            new_label = self.graph['region'].nodes[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=reg, action=act, marker='unvisited')
            if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
                self.graph['initial'].add(prod_node)
        return prod_node    
    
    def projection(self, prod_node):
        reg = self.nodes[prod_node]['region']
        act = self.nodes[prod_node]['action']
        return reg, act

class MotActModel2(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set(), type='MotActModel')

    # creates the states for the complete model
    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            new_label = self.graph['region'].nodes[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=self.graph['region'].nodes[reg]['pose'], action=act, marker='unvisited')
            if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
                self.graph['initial'].add(prod_node)
        return prod_node 
    
    def build_full(self):
        for reg in self.graph['region'].nodes():
            for act in self.graph['action'].action.keys():
                # adding actions transitions
                for act_to in self.graph['action'].allowed_actions(reg):
                    if act_to != 'None':
                        # create action node
                        prod_node_to = self.composition(reg, act_to)
                        if(act=='None'):
                            # add edge from None to action
                            self.add_edge((reg, act), prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                        # add edge from action to None
                        self.add_edge(prod_node_to, (reg, 'None'), weight=self.graph['action'].action['None'][0], label= 'none', marker= 'visited')
                # adding motions transitions
                for reg_to in self.graph['region'].successors(reg):
                    prod_node_to = self.composition(reg_to, 'None')
                    # add edge between regions (actions None for both nodes)
                    self.add_edge((reg, 'None'), prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto_'+str(reg_to), marker= 'visited')
            # adding self transitions (none action to waste time)
            self.add_edge((reg, 'None'), (reg, 'None'), weight=self.graph['action'].action['None'][0], label= 'none', marker= 'visited')                
            
                
                
                



   
        """
        
    def build_initial(self):
        for reg_init in self.graph['region'].graph['initial']:
            init_prod_node = self.composition(reg_init, 'None')
    
    def fly_successors_iter(self, prod_node): 
        reg, act = self.projection(prod_node)
        # been visited before, and hasn't changed 
        if ((self.node[prod_node]['marker'] == 'visited') and 
            (self.graph['region'].node[self.node[prod_node]['region']]['status'] == 'confirmed')):
            for prod_node_to in self.successors_iter(prod_node):
                yield prod_node_to, self.edge[prod_node][prod_node_to]['weight']
        else:
            self.remove_edges_from(self.out_edges(prod_node))
            # actions 
            label = self.graph['region'].node[reg]['label']
            for act_to in self.graph['action'].allowed_actions(label):
                if act_to != 'None':
                    prod_node_to = self.composition(reg, act_to)
                    cost = self.graph['action'].action[act_to][0]
                    self.add_edge(prod_node, prod_node_to, weight=cost, label= act_to)
                    yield prod_node_to, cost
            # motions
            for reg_to in self.graph['region'].successors_iter(reg):
                prod_node_to = self.composition(reg_to, 'None')
                cost = self.graph['region'][reg][reg_to]['weight']
                self.add_edge(prod_node, prod_node_to, weight=cost, label= 'goto')         
                yield prod_node_to, cost
            self.graph['region'].node[self.node[prod_node]['region']]['status'] = 'confirmed'
            self.node[prod_node]['marker'] = 'visited'

    def fly_predecessors_iter(self, prod_node): 
        reg, act = self.projection(prod_node)
        # actions
        label = self.graph['region'].node[reg]['label']
        if (act in self.graph['action'].allowed_actions(label)) and (act != 'None'):    
            for f_act in self.graph['action'].action.iterkeys():
                f_prod_node = self.composition(reg, f_act)
                cost = self.graph['action'].action[act][0]
                self.add_edge(f_prod_node, prod_node, weight=cost, label= act)
                yield f_prod_node, cost
        # motions
        if act == 'None':
            for f_reg in self.graph['region'].predecessors_iter(reg):
                if f_reg !=reg:
                    for f_act in self.graph['action'].action.iterkeys():
                            f_prod_node = self.composition(f_reg, f_act)
                            cost = self.graph['region'][f_reg][reg]['weight']
                            self.add_edge(f_prod_node, prod_node, weight=cost, label= 'goto')         
                            yield f_prod_node, cost
        
        
        """
    
    
    
    
    
    