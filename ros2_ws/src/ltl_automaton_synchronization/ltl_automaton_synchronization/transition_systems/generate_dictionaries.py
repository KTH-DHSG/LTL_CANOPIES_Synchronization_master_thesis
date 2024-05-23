import yaml
import os
from ament_index_python.packages import get_package_prefix
import math
#TODOD: if there is time create a file that asks the user to input and create the dictionary
# auxilliary functions
# write_to_file(file_name, dict) writes the dictionary to a yaml file

    

def write_to_file(file_name="dictionary", dict={}):
    with open(os.path.join(get_package_prefix('ltl_automaton_synchronization').replace('/install/', '/src/'),
                                                                            'ltl_automaton_synchronization',
                                                                             'config',
                                                                             file_name+'.yaml'),'w') as file:
        yaml.dump(dict, file, sort_keys=False)

    print("-----------------------")
    print(" Generation Successful!")
    print("-----------------------")
    print(" File can be find at %s" % os.path.join(get_package_prefix('ltl_automaton_synchronization').replace('/install/', '/src/'),
                                                                            'ltl_automaton_synchronization',
                                                                             'config',
                                                                             file_name+'.yaml'))


# output_standard() generates a standard dictionary for testing purposes
def output_standard(file_name, robot_type):
    if robot_type=='rosie':
        ROBOT_SPEED = 0.5
        def dist(point1, point2):
            return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)/ROBOT_SPEED
        # ROI = (x, y, radius)
        H = [0, 0, 0.7]
        D = [0.55, -2.15, 0.45]
        M1 = [-0.35, 1.45, 0.45]
        M2 = [1.45, 0.45, 0.45]
        M3 = [-1.55, -0.35, 0.45]
        M4 = [-1.95, -0.85, 0.45]
        weights=[[dist(H,H), dist(H,D), dist(H,M1), dist(H,M2), dist(H,M3), dist(H,M4)],
                 [dist(D,H), dist(D,D), dist(D,M1), dist(D,M2), dist(D,M3), dist(D,M4)],
                 [dist(M1,H), dist(M1,D), dist(M1,M1), dist(M1,M2), dist(M1,M3), dist(M1,M4)],
                 [dist(M2,H), dist(M2,D), dist(M2,M1), dist(M2,M2), dist(M2,M3), dist(M2,M4)],
                 [dist(M3,H), dist(M3,D), dist(M3,M1), dist(M3,M2), dist(M3,M3), dist(M3,M4)],
                 [dist(M4,H), dist(M4,D), dist(M4,M1), dist(M4,M2), dist(M4,M3), dist(M4,M4)]
                ]

        region_dict = {
            'initial': 'h',
            'type': 'MotionTS',
            'regions':{
                'h' : {'label':'h', 'pose': H, 'weights':weights[0]},
                'd' : {'label':'d', 'pose': D, 'weights':weights[1]},
                'm1': {'label':'m1','pose': M1, 'weights':weights[2]},
                'm2': {'label':'m2','pose': M2, 'weights':weights[3]},
                'm3': {'label':'m3','pose': M3, 'weights':weights[4]},
                'm4': {'label':'m4','pose': M4, 'weights':weights[5]}
            }
        }
        # action_dict = {act_name: {cost, guard_formula, label, dependendy : {}, type}} 
        action_dict = {
            'type': 'ActionModel',
            'initial':'free', #free=none
            'actions':{
                'load': {'weight':20, 'guard':'h', 'label': 'load', "dependency": {'h_load':['h']}, 'type': 'collaborative'},
                'h_load': {'weight':20, 'guard':'h', 'label': 'h_load', "dependency": {}, 'type': 'assisting'},
                'manipulate': {'weight':15, 'guard':'m1 || m2 || m3 || m4', 'label': 'manipluate', "dependency": {}, 'type': 'local'},
                'deliver': {'weight':30, 'guard':'d', 'label': 'deliver', "dependency": {}, 'type': 'local'},
                'harvest': {'weight':40, 'guard':'h', 'label': 'harvest', "dependency": {}, 'type': 'local'},
                'free' : {"weight":1, "guard":'1', "label": 'none', "dependency":{}, "type": 'local'}        
            }
        }
    elif robot_type=='turtlebot':        
        ROBOT_SPEED = 0.2
        def dist(point1, point2):
            return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)/ROBOT_SPEED
        
       # ROI = (x, y, radius)
        P1 = [-1.55, 1.05, 0.3]
        P2 = [0.85, 1.45, 0.3]
        P3 = [-0.15, -2.65, 0.3]
        P4 = [1.35, -1.95, 0.3]
        C1 = [-2.05, 1.95, 0.3]
        C2 = [1.75, 1.95, 0.3]
        C3 = [-2.05, -2.65, 0.3]
        C4 = [7.75, -2.65, 0.3]
        weights=[[dist(P1,P1), dist(P1,P2), dist(P1,P3), dist(P1,P4), dist(P1,C1), dist(P1,C2), dist(P1,C3), dist(P1,C4)],
                 [dist(P2,P1), dist(P2,P2), dist(P2,P3), dist(P2,P4), dist(P2,C1), dist(P2,C2), dist(P2,C3), dist(P2,C4)],
                 [dist(P3,P1), dist(P3,P2), dist(P3,P3), dist(P3,P4), dist(P3,C1), dist(P3,C2), dist(P3,C3), dist(P3,C4)],
                 [dist(P4,P1), dist(P4,P2), dist(P4,P3), dist(P4,P4), dist(P4,C1), dist(P4,C2), dist(P4,C3), dist(P4,C4)],
                 [dist(C1,P1), dist(C1,P2), dist(C1,P3), dist(C1,P4), dist(C1,C1), dist(C1,C2), dist(C1,C3), dist(C1,C4)],
                 [dist(C2,P1), dist(C2,P2), dist(C2,P3), dist(C2,P4), dist(C2,C1), dist(C2,C2), dist(C2,C3), dist(C2,C4)],
                 [dist(C3,P1), dist(C3,P2), dist(C3,P3), dist(C3,P4), dist(C3,C1), dist(C3,C2), dist(C3,C3), dist(C3,C4)],
                 [dist(C4,P1), dist(C4,P2), dist(C4,P3), dist(C4,P4), dist(C4,C1), dist(C4,C2), dist(C4,C3), dist(C4,C4)]
                ]
        region_dict = {
            'initial': 'h',
            'type': 'MotionTS',
            'regions':{
                'p1' : {'label':'p1', 'pose': P1, 'weights':weights[0]},
                'p2' : {'label':'p2', 'pose': P2, 'weights':weights[1]},
                'p3': {'label':'p3','pose': P3, 'weights':weights[2]},
                'p4': {'label':'p4','pose': P4, 'weights':weights[3]},
                'c1': {'label':'c1','pose': C1, 'weights':weights[4]},
                'c2': {'label':'c2','pose': C2, 'weights':weights[5]},
                'c3': {'label':'c3','pose': C3, 'weights':weights[6]},
                'c4': {'label':'c4','pose': C4, 'weights':weights[7]}
            }
        }
        # action_dict = {act_name: {cost, guard_formula, label, dependendy : {}, type}} 
        action_dict = {
            'type': 'ActionModel',
            'initial':'free', #free=none
            'actions':{
                'check_connection': {'weight':30, 'guard':'c1 || c3', 'label': 'check_connection', "dependency": {'h_check_connection':['c2', 'c4']}, 'type': 'collaborative'},
                'h_check_connection': {'weight':25, 'guard':'c2 || c4', 'label': 'h_check_connection', "dependency": {}, 'type': 'assisting'},
                'patrol': {'weight':20, 'guard':'p1 || p2 || p3 || p4', 'label': 'patrol', "dependency": {}, 'type': 'local'},
                'free' : {"weight":1, "guard":'1', "label": 'none', "dependency":{}, "type": 'local'}        
            }
        }
    else:
        print("Robot type not recognized")
        return
    
    
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
print("Please enter robot type")
prompt = '> '
robot_type = input(prompt)
output_standard(file_name, robot_type)














