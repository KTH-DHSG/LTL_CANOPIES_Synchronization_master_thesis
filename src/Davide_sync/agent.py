
from motion_ts import MotionFts, MotionFts2
from action_mod import ActionModel
from motion_action import MotActModel, MotActModel2
import networkx as nx
import matplotlib.pyplot as plt
from parser import parse as parse_guard

# ROI = (x, y, radius)
# residential regions
R1 = (15, 45, 3)
R2 = (0, 20, 3)
R3 = (15, 10, 3)

R_dict = {    
    R1: set(['r1']),
    R2: set(['r2']),    
    #R3: set(['r3'])   
}
R_symbols = set(['r1','r2'])#,'r3'])
r_dict2= {
    'r1': (set(['r1']), R1),
    'r2': (set(['r2']), R2),
    'r3': (set(['r3']), R3)
}
init_pose = (0, 20, 1)
dep= dict()
dep['c']= set(['a'])
#print(dep)
dep['m']= set(['b'])
#print(dep)

# transitions cost must be defined as a matrix of size len(R_dict) x len(R_dict)
weights=[[0, 2, 5], [4, 0, 7], [2, 3, 0]]
weights1=[[0, 2], [4, 0]]

motion = MotionFts(R_dict, R_symbols, 'a11_FTS')
#print("Nodes:\n"+str(motion.nodes(data=True)))
motion.add_full_edges(cost=weights1)
#print(motion.closest_node(init_pose))
#print(motion.edges(data=True))
motion.set_initial(init_pose)


#motion = MotionFts(R_dict, R_symbols, 'a11_FTS')
motion2 = MotionFts2(r_dict2, set(r_dict2.keys()), 'a11_FTS')
#print("Nodes:\n"+str(motion.nodes(data=True)))
motion2.add_full_edges(cost=weights)
#print(motion.closest_node(init_pose))
#print(motion.edges(data=True))
motion2.set_initial(init_pose)

#D= motion
#pos = nx.spring_layout(D)
#fig, ax = plt.subplots(figsize=(10, 5))

#edges_weights = [data['weight'] for _, _, data in D.edges(data=True)]
#nx.draw_networkx_nodes(D, pos, ax=ax, node_size=500, node_color="#acddc5", edgecolors='g')
#nx.draw_networkx_labels(D, pos, ax=ax, font_weight='bold', font_size=12)
#nx.draw_networkx_edges(D, pos, ax=ax, edgelist=edges_weights, edge_color="g")
#nx.draw_networkx_edges(D, pos, ax=ax, edgelist=D.edges(data=True), edge_color="g", connectionstyle="arc3,rad=0.1")
#nx.draw_networkx_edge_labels(D, pos, ax=ax, edge_labels={(u, v): d['weight'] for u, v, d in D.edges(data=True)}, label_pos=0.33)
#print({(u, v): d['weight'] for u, v, d in D.edges(data=True)})
#print(list(motion.nodes.data()))
#print(motion.set_initial(init_pose))
#print("\n Edges:\n"+str(motion.edges.data()))
#nx.draw(motion)
#nx.draw_networkx_edge_labels(motion)
#plt.draw()
#plt.show()
#print("Press any key to continue...")
#input()
#print("Continuing execution...")
#print(motion.get_edge_data(R3,R3))




# action_dict = {act_name: (cost, guard_formula, label, dependendy, type)}
action_dict = {
    'c13': (10, 'r1 || r3', set(['p41']), {'h1':set(['r1','r3']), 'h2':set(['r2'])}, 'collaborative'),
    'l123': (20, '1', set(['d41']), {}, 'local'),
    'a2': (30, 'r2', set(['p42']), {}, 'assisting')
}
actions = ActionModel(action_dict)
#print(actions.allowed_actions({'r3', 'r1'}))

# to check if a dictionry is empty use bool(dict} if false the dictionary is empty

model = MotActModel(motion, actions)

model2 = MotActModel2(motion2, actions)
#print(model.composition(R1, 'p41'))
#print(model2.composition('r1', 'p41'))
#print(model2.graph['action'].action['p41'])
#print(model.nodes(data=True))
#print(model2.nodes(data=True))
#print(model.graph['action'].action.keys())
#print(motion.nodes.data())
#print(model.projection(model.composition(R1, 'p41')))
#print(model.graph['action'].action)
#action[act][2]
#print('successors')
#for succ in model2.graph['region'].successors('r1'):
#    print(succ)
#print('---')
model2.build_full()
#print(model2.graph['region'].nodes())
for edge in model2.edges(data=True):
    print(edge)
#print(model2)

D= model2
#D = motion2
pos = nx.spring_layout(D)
fig, ax = plt.subplots(figsize=(10, 5))

edges_weights = [data['weight'] for _, _, data in D.edges(data=True)]
nx.draw_networkx_nodes(D, pos, ax=ax, node_size=500, node_color="#acddc5", edgecolors='g')
nx.draw_networkx_labels(D, pos, ax=ax, font_weight='bold', font_size=12)
nx.draw_networkx_edges(D, pos, ax=ax, edgelist=D.edges(data=True), edge_color="r", connectionstyle="arc3,rad=0.1")
nx.draw_networkx_edge_labels(D, pos, ax=ax, edge_labels={(u, v): d['label'] for u, v, d in D.edges(data=True)}, label_pos=0.33)
plt.show()
