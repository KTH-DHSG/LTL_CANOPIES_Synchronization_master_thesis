
from ros2_ws.src.ltl_automaton_synchronization.ltl_automaton_synchronization.transition_systems.old.motion_ts import MotionFts, MotionFts2, MotionFts3
from ros2_ws.src.ltl_automaton_synchronization.ltl_automaton_synchronization.transition_systems.old.action_mod import ActionModel, ActionModel2
from ros2_ws.src.ltl_automaton_synchronization.ltl_automaton_synchronization.transition_systems.old.motion_action import MotActModel, MotActModel2, MotActModel3
import networkx as nx
import matplotlib.pyplot as plt
import yaml
#from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard
from ltl_automaton_planner.nodes.planner_node import show_automaton
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


R11 = [15, 45, 3]
R22 = [0, 20, 3]
R33 = [15, 10, 3]
weights=[[0, 2, 5], [4, 0, 7], [2, 3, 0]]
r_dict2_mod = {
    'r1': {'label':'r1','pose': R11, 'weights':weights[0]},
    'r2': {'label':'r2','pose': R22, 'weights':weights[1]},
    'r3': {'label':'r3','pose': R33, 'weights':weights[2]}
}
with open('region_dict_mod.yaml', 'w') as file:
    yaml.dump(r_dict2_mod, file)
#print(r_dict2_mod)
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
#motion.set_initial(init_pose)

motion2_mod = MotionFts3(r_dict2_mod, set(r_dict2_mod.keys()), 'a11_FTS')
motion2_mod.add_full_edges()
motion2_mod.set_initial(init_pose)
#motion = MotionFts(R_dict, R_symbols, 'a11_FTS')
motion2 = MotionFts2(r_dict2, set(r_dict2.keys()), 'a11_FTS')
#print(motion2_mod.nodes(data=True))
#print(motion2_mod.nodes(data=True)['r1']['pose'][1])
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
action_dict_mod = {
    'c13': {'weight':10, 'guard':'r1 || r3', 'label': 'p41', "dependency": {'h1':['r1','r3'], 'h2':['r2']}, 'type': 'collaborative'},
    'l123': {'weight':20, 'guard':'1', 'label': 'd41', "dependency": {}, 'type': 'local'},
    'a2': {'weight':30, 'guard':'r2', 'label': 'p42', "dependency": {}, 'type': 'assisting'}
}
with open('action_dict_mod.yaml', 'w') as file:
    yaml.dump(action_dict_mod, file)
    
with open('action_dict_mod.yaml', 'r') as file:
    action_dict_mod_loaded = yaml.safe_load(file)

#print(action_dict_mod==action_dict_mod_loaded)

#print(action_dict_mod_loaded)
actions = ActionModel(action_dict)
actions2 = ActionModel2(action_dict_mod)
#print(actions.allowed_actions({'r3', 'r1'}))
#print(actions.action)
#print(actions2.action)
# to check if a dictionry is empty use bool(dict} if false the dictionary is empty

model = MotActModel(motion, actions)

model2 = MotActModel2(motion2, actions)

model3 = MotActModel3(motion2_mod, actions2)

model3_5 = MotActModel3(motion2_mod, actions2)

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
model3.build_full()
model3_5.build_nodes()
model3_5.build_edges()
print(model3_5.graph['initial'])
#print(model3.nodes(data=True))
#print(model2.graph['region'].nodes())
#for edge in model2.edges(data=True):
    #print(edge)
#print(model2)
#print(model2.edges(data=True))
#print(motion2_mod.edges(data=True))
#D= motion2_mod
D= model3_5
#print(D.edges(data=True))
#D = motion2
pos = nx.circular_layout(D)
fig, axs = plt.subplots(1,2,figsize=(100, 50))

#edges_weights = [data['weight'] for _, _, data in D.edges(data=True)]
nx.draw_networkx_nodes(D, pos, ax=axs[0], node_size=500, node_color="#acddc5", edgecolors='g')
nx.draw_networkx_labels(D, pos, ax=axs[0], font_weight='bold', font_size=12)
nx.draw_networkx_edges(D, pos, ax=axs[0], edgelist=D.edges(data=True), edge_color="r", connectionstyle="arc3,rad=0.1")
nx.draw_networkx_edge_labels(D, pos, ax=axs[0], edge_labels={(u, v): d['action'] for u, v, d in D.edges(data=True)}, label_pos=0.33)


D= model3
#print(D.edges(data=True))
#D = motion2
pos = nx.circular_layout(D)

#edges_weights = [data['weight'] for _, _, data in D.edges(data=True)]
nx.draw_networkx_nodes(D, pos, ax=axs[1], node_size=500, node_color="#acddc5", edgecolors='g')
nx.draw_networkx_labels(D, pos, ax=axs[1], font_weight='bold', font_size=12)
nx.draw_networkx_edges(D, pos, ax=axs[1], edgelist=D.edges(data=True), edge_color="r", connectionstyle="arc3,rad=0.1")
nx.draw_networkx_edge_labels(D, pos, ax=axs[1], edge_labels={(u, v): d['action'] for u, v, d in D.edges(data=True)}, label_pos=0.33)
#plt.show()

#show_automaton(model2)

