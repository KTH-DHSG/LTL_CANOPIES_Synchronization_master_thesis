
import networkx as nx
import matplotlib.pyplot as plt
import yaml
import os
from ament_index_python.packages import get_package_prefix

from ts_definitions import MotionTS, ActionModel, MotActTS



file_name = 'region_dictionary'
region_dict = {}
with open(os.path.join(get_package_prefix('ltl_automaton_synchronization').replace('/install/', '/src/'),
                                                                        'ltl_automaton_synchronization',
                                                                            'config',
                                                                            file_name+'.yaml'),'r') as file:
    region_dict = yaml.safe_load(file)
    
file_name = 'action_dictionary'

action_dict = {}
with open(os.path.join(get_package_prefix('ltl_automaton_synchronization').replace('/install/', '/src/'),
                                                                        'ltl_automaton_synchronization',
                                                                            'config',
                                                                            file_name+'.yaml'),'r') as file:
    action_dict = yaml.safe_load(file)

motion_ts = MotionTS(region_dict, set(region_dict.keys()), 'a11_FTS')
action_mod = ActionModel(action_dict)
model = MotActTS(motion_ts, action_mod)
model.build_full()
for edge in model.edges(data=True):
    print (edge)


D= motion_ts

pos = nx.circular_layout(D)
fig, axs = plt.subplots(1,2,figsize=(100, 50))

nx.draw_networkx_nodes(D, pos, ax=axs[0], node_size=500, node_color="#acddc5", edgecolors='g')
nx.draw_networkx_labels(D, pos, ax=axs[0], font_weight='bold', font_size=12)
nx.draw_networkx_edges(D, pos, ax=axs[0], edgelist=D.edges(data=True), edge_color="r", connectionstyle="arc3,rad=0.1")
nx.draw_networkx_edge_labels(D, pos, ax=axs[0], edge_labels={(u, v): d['weight'] for u, v, d in D.edges(data=True)}, label_pos=0.33)

D= model
pos = nx.circular_layout(D)

nx.draw_networkx_nodes(D, pos, ax=axs[1], node_size=500, node_color="#acddc5", edgecolors='g')
nx.draw_networkx_labels(D, pos, ax=axs[1], font_weight='bold', font_size=12)
nx.draw_networkx_edges(D, pos, ax=axs[1], edgelist=D.edges(data=True), edge_color="r", connectionstyle="arc3,rad=0.1")
nx.draw_networkx_edge_labels(D, pos, ax=axs[1], edge_labels={(u, v): d['action'] for u, v, d in D.edges(data=True)}, label_pos=0.33)
plt.show()


