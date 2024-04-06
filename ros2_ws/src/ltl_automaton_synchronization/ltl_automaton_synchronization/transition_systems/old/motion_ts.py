from math import sqrt
from networkx import DiGraph

def distance(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


class MotionFts(DiGraph):
    
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        for (n, label) in node_dict.items():
            self.add_node(n, label=label)# status='confirmed' removed since it was used to consider only close neighbors

    # add edges to the graph
    def add_full_edges(self, cost):
        i=0
        for i_node in self.nodes():
            j=0
            for j_node in self.nodes():
                if (i_node, j_node) not in self.edges():
                    self.add_edge(i_node, j_node, weight= cost[i][j])
                j+=1
            i+=1

    # set the initial node of the graph    
    def set_initial(self, pose):                
        init_node = self.closest_node(pose)
        self.graph['initial'] = set([init_node])
        return init_node

    # find the closest node to the given pose if it is in the graph
    def closest_node(self, pose):        
        # check if the pose is in the graph       
        possible_nodes = []
        for node in self.nodes():
            if distance(node,pose) <= node[2]:
                possible_nodes.append(node)
        # if the pose is not in the graph, raise an exception
        if len(possible_nodes) == 0:
            raise Exception("No node found for the given pose")
        # if the pose is in the graph, return the closest node
        node = min(possible_nodes, key= lambda n: distance(n,pose))        
        return node


#IMPORTANTD: not sure what symbols are fore so leave it like that for now but may be removed
   

class MotionFts2(DiGraph):
    
    # node_dict = { symbol : (label, pose)}    
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        for (n, entry) in node_dict.items():
            self.add_node(n, label=entry[0], pose=entry[1] )
            
    # add edges to the graph
    def add_full_edges(self, cost):
        i=0
        for i_node in self.nodes():
            j=0
            for j_node in self.nodes():
                if (i_node, j_node) not in self.edges():
                    if(i_node != j_node):
                        self.add_edge(i_node, j_node, weight = cost[i][j])
                j+=1
            i+=1

    # set the initial node of the graph    
    def set_initial(self, pose):                
        init_node = self.closest_node(pose)
        self.graph['initial'] = set([init_node])
        return init_node

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
            # TODOD:change from Exception to return None
            raise Exception("No node found for the given pose")
        # if the pose is in the graph, return the closest node
        node = min(possible_nodes, key= lambda n: distance(self.nodes[n]['pose'],pose))        
        return node
    
    
class MotionFts3(DiGraph):
    
    # node_dict = { symbol : (label, pose)}    
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        for (n, attrib) in node_dict.items():
            self.add_node(n, label=attrib['label'], pose=attrib['pose'], weights=attrib['weights'] )
            
    # add edges to the graph
    def add_full_edges(self):
        for (i_node, i_data) in self.nodes(data=True):
            j=0
            for j_node in self.nodes():
                if (i_node, j_node) not in self.edges():
                    if(i_node != j_node):
                        self.add_edge(i_node, j_node, weight = i_data['weights'][j])
                j+=1

    # set the initial node of the graph    
    def set_initial(self, pose):                
        init_node = self.closest_node(pose)
        self.graph['initial'] = set([init_node])
        return init_node

    # find the closest node to the given pose if it is in the graph
    def closest_node(self, pose):        
        # check if the pose is in the graph       
        possible_nodes = []
        for (node, data) in self.nodes(data=True):
            node_pose = data['pose']
            print(node_pose)
            if distance(node_pose,pose) <= node_pose[2]:
                possible_nodes.append(node)
        # if the pose is not in the graph, raise an exception
        if len(possible_nodes) == 0:
            # TODOD:change from Exception to return None
            raise Exception("No node found for the given pose")        
        # if the pose is in the graph, return the closest node        
        node = min(possible_nodes, key= lambda n: distance(self.nodes[n]['pose'],pose))       
        return node

        
        """
    #IMPORTANTD: probably not needed becasue we don't consider obstacle regions
    def add_un_edges(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_node = edge[0]
            t_node = edge[1]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)

    # IMPORTANTD:this part is probbly not needed if we don't modify the regions but consider them static
    def update_after_region_change(self, sense_info, com_info, margin=10):
        # sense_info = {'label':set((x,y), l', l'_)), 'edge':(set(add_edges), set(del_edges))}
        # com_info = set((x,y), l', l'_))
        # margin for adding new nodes, NOTE units!
        changed_regs = set()
        # label udpate
        label_info = sense_info['label']
        label_info.update(com_info)
        for mes in label_info:
            if mes[1]:
                close_node = self.closest_node(mes[0])
                if distance(close_node, mes[0])>margin:
                    self.add_node(mes[0], mes[1])
                else:
                    old_label = self.node[close_node]['label']
                    new_label = old_label.union(mes[1]).difference(mes[2])
                    if old_label != new_label:
                        self.node[close_node]['label'] = set(new_label)
                        self.node[close_node]['status'] = 'notconfirmed'
                        changed_regs.add(close_node)
        # edges udpate
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            self.add_edge(e[0], e[1], weight=distance(e[0], e[1]))
            self.node[close_node]['status'] = 'notconfirmed'
            changed_regs.add(e[0])
        for e in edge_info[1]:
            self.remove_edge(e[0], e[1])
            changed_regs.add(e[0])
            self.node[close_node]['status'] = 'notconfirmed'
        return chnaged_regs
        
        
        """
    