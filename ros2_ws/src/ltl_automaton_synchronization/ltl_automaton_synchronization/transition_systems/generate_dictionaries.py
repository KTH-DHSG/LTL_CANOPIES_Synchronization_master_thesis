import yaml
import os
from ament_index_python.packages import get_package_prefix
#TODOD: if there is time create a file that asks the user to input and create the dictionary
# auxilliary functions
# write_to_file(file_name, dict) writes the dictionary to a yaml file
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

# output_standard() generates a standard dictionary for testing purposes
def output_standard(file_name):
    # ROI = (x, y, radius)
    R1 = [15, 45, 3]
    R2 = [0, 20, 3]
    R3 = [15, 10, 3]
    weights=[[0, 2, 5], [4, 0, 7], [2, 3, 0]]
    region_dict = {
        'initial': 'r1',
        'type': 'MotionTS',
        'regions':{
            'r1': {'label':'r1','pose': R1, 'weights':weights[0]},
            'r2': {'label':'r2','pose': R2, 'weights':weights[1]},
            'r3': {'label':'r3','pose': R3, 'weights':weights[2]}
        }
    }
    # action_dict = {act_name: {cost, guard_formula, label, dependendy : {}, type}} 
    action_dict = {
        'type': 'ActionModel',
        'initial':'free', #free=none
        'actions':{
            'c13': {'weight':10, 'guard':'r1 || r3', 'label': 'c13', "dependency": {'h1':['r1','r3'], 'h2':['r2']}, 'type': 'collaborative'},
            'l123': {'weight':20, 'guard':'1', 'label': 'l123', "dependency": {}, 'type': 'local'},
            'a2': {'weight':30, 'guard':'r2', 'label': 'a2', "dependency": {}, 'type': 'assisting'},
            'free' : {"weight":1, "guard":'1', "label": '', "dependency":{}, "type": 'local'}        
        }
    }
    full_dictionary = {
        'motion': region_dict,
        'action': action_dict
    }
    write_to_file(file_name, full_dictionary)

print("----------------------------------------")
print(" Motion and Action Dictionary Generator ")
print("----------------------------------------")
print("Please enter file name")
prompt = '> '
file_name = input(prompt)
output_standard(file_name)














