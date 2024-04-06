import yaml
import os
from ament_index_python.packages import get_package_prefix


def write_to_file(file_name="dictionary", dict={}):
    with open(os.path.join(get_package_prefix('ltl_automaton_synchronization').replace('/install/', '/src/'),
                                                                            'ltl_automaton_synchronization',
                                                                             'config',
                                                                             file_name+'.yaml'),'w') as file:
        yaml.dump(dict, file)

    print("-----------------------")
    print(" Generation Successful!")
    print("-----------------------")
    print(" File can be find at %s" % os.path.join(get_package_prefix('ltl_automaton_synchronization').replace('/install/', '/src/'),
                                                                            'ltl_automaton_synchronization',
                                                                             'config',
                                                                             file_name+'.yaml'))




# Fixed dictionaries for basic testing
# ROI = (x, y, radius)
R1 = [15, 45, 3]
R2 = [0, 20, 3]
R3 = [15, 10, 3]
weights=[[0, 2, 5], [4, 0, 7], [2, 3, 0]]
region_dict = {
    'r1': {'label':'r1','pose': R1, 'weights':weights[0]},
    'r2': {'label':'r2','pose': R2, 'weights':weights[1]},
    'r3': {'label':'r3','pose': R3, 'weights':weights[2]}
}
# action_dict = {act_name: {cost, guard_formula, label, dependendy : {}, type}} 
action_dict = {
    'c13': {'weight':10, 'guard':'r1 || r3', 'label': 'p41', "dependency": {'h1':['r1','r3'], 'h2':['r2']}, 'type': 'collaborative'},
    'l123': {'weight':20, 'guard':'1', 'label': 'd41', "dependency": {}, 'type': 'local'},
    'a2': {'weight':30, 'guard':'r2', 'label': 'p42', "dependency": {}, 'type': 'assisting'}
}

write_to_file("region_dictionary", region_dict)
write_to_file("action_dictionary", action_dict)









