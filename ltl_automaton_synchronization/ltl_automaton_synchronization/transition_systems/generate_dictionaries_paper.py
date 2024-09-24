import yaml
import os
from ament_index_python.packages import get_package_prefix
import math
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
        ROBOT_SPEED = 0.2
        def dist(point1, point2):
            return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)/ROBOT_SPEED
        # ROI = (x, y, radius)  
        L = [1.256, 1.471, 0.69]
        M = [1.249, 0.001, 0.59]
        S = [0.166, 0.982, 0.4]
        D = [-1.205, 0.819, 0.60]
        H1 = [-1.882, 1.720, 0.45]
        H2 = [-0.580, 1.722, 0.45]
        H3 = [-0.536, -0.060, 0.45]
        H4 = [-1.831, -0.073, 0.45]       
    
        weights=[[dist(H1,H1), dist(H1,H2), dist(H1,H3), dist(H1,H4), dist(H1,M), dist(H1,S), dist(H1,D), dist(H1,L)],
                 [dist(H2,H1), dist(H2,H2), dist(H2,H3), dist(H2,H4), dist(H2,M), dist(H2,S), dist(H2,D), dist(H2,L)],
                 [dist(H3,H1), dist(H3,H2), dist(H3,H3), dist(H3,H4), dist(H3,M), dist(H3,S), dist(H3,D), dist(H3,L)],
                 [dist(H4,H1), dist(H4,H2), dist(H4,H3), dist(H4,H4), dist(H4,M), dist(H4,S), dist(H4,D), dist(H4,L)],
                 [dist(M,H1), dist(M,H2), dist(M,H3), dist(M,H4), dist(M,M), dist(M,S), dist(M,D), dist(M,L)],
                 [dist(S,H1), dist(S,H2), dist(S,H3), dist(S,H4), dist(S,M), dist(S,S), dist(S,D), dist(S,L)],
                 [dist(D,H1), dist(D,H2), dist(D,H3), dist(D,H4), dist(D,M), dist(D,S), dist(D,D), dist(D,L)],
                 [dist(L,H1), dist(L,H2), dist(L,H3), dist(L,H4), dist(L,M), dist(L,S), dist(L,D), dist(L,L)]
                ]    
                 
        region_dict = {
            'initial': 'h1',
            'type': 'MotionTS',
            'regions':{
                'h1' : {'label':'h1', 'pose': H1, 'weights':weights[0]},
                'h2' : {'label':'h2', 'pose': H2, 'weights':weights[1]},
                'h3' : {'label':'h3', 'pose': H3, 'weights':weights[2]},
                'h4' : {'label':'h4', 'pose': H4, 'weights':weights[3]},
                'm' : {'label':'m','pose': M, 'weights':weights[4]},
                's' : {'label':'s','pose': S, 'weights':weights[5]},
                'd' : {'label':'d','pose': D, 'weights':weights[6]},
                'l' : {'label':'l','pose': L, 'weights':weights[7]},
            }
        }
        # action_dict = {act_name: {cost, guard_formula, label, dependendy : {}, type}} 
        action_dict = {
            'type': 'ActionModel',
            'initial':'free', #free=none
            'actions':{
                'load': {'weight':6, 'guard':'l', 'label': 'load', "dependency": {'h_load':['l']}, 'type': 'collaborative'},
                'h_load': {'weight':6, 'guard':'l', 'label': 'h_load', "dependency": {}, 'type': 'assisting'},
                'h_remove_object': {'weight':60, 'guard':'m', 'label': 'h_remove_object', "dependency": {}, 'type': 'assisting'},
                'harvest': {'weight':5, 'guard':'h1 || h2 || h3 || h4', 'label': 'harvest', "dependency": {}, 'type': 'local'},
                'manipulate': {'weight':8, 'guard':'m', 'label': 'manipulate', "dependency": {}, 'type': 'local'},
                'deliver': {'weight':5, 'guard':'d', 'label': 'deliver', "dependency": {}, 'type': 'local'},
                'supervise': {'weight':6, 'guard':'s', 'label': 'supervise', "dependency": {}, 'type': 'local'},
                'wait' : {"weight":20, "guard":'1', "label": 'wait', "dependency":{}, "type": 'local'},
                'free' : {"weight":1, "guard":'1', "label": 'none', "dependency":{}, "type": 'local'}        
            }
        }
    elif robot_type=='turtlebot':        
        ROBOT_SPEED = 0.15
        def dist(point1, point2):
            return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)/ROBOT_SPEED
        
       # ROI = (x, y, radius)
        P1 = [-1.195, -0.868, 0.19]
        P2 = [-0.488, -0.871, 0.19]
        P3 = [0.205, -0.871, 0.19]
        P4 = [0.869, -0.863, 0.19]
        P5 = [1.551, -1.430, 0.19]
        P6 = [1.538, -2.020, 0.19]
        P7 = [0.858, -2.594, 0.19]
        P8 = [0.189, -2.592, 0.19]
        P9 = [-0.488, -2.587, 0.19]
        P10 = [-1.148, -2.615, 0.19]
        P11 = [-1.826, -2.028, 0.19]
        P12 = [-1.851, -1.470, 0.19]
        C1 = [-1.856, -0.886, 0.19]
        C2 = [1.549, -0.818, 0.19]
        C3 = [1.557, -2.568, 0.19]
        C4 = [-1.803, -2.608, 0.19]
        G  = [-0.142, -1.679, 0.44]
        M  = [1.249, 0.001, 0.59]    
        weights=[[dist(P1,P1), dist(P1,P2), dist(P1,P3), dist(P1,P4), dist(P1,P5), dist(P1,P6), dist(P1,P7), dist(P1,P8), dist(P1,P9), dist(P1,P10), dist(P1,P11), dist(P1,P12), dist(P1,C1), dist(P1,C2), dist(P1,C3), dist(P1,C4), dist(P1,G), dist(P1,M)],
                 [dist(P2,P1), dist(P2,P2), dist(P2,P3), dist(P2,P4), dist(P2,P5), dist(P2,P6), dist(P2,P7), dist(P2,P8), dist(P2,P9), dist(P2,P10), dist(P2,P11), dist(P2,P12), dist(P2,C1), dist(P2,C2), dist(P2,C3), dist(P2,C4), dist(P2,G), dist(P2,M)],
                 [dist(P3,P1), dist(P3,P2), dist(P3,P3), dist(P3,P4), dist(P3,P5), dist(P3,P6), dist(P3,P7), dist(P3,P8), dist(P3,P9), dist(P3,P10), dist(P3,P11), dist(P3,P12), dist(P3,C1), dist(P3,C2), dist(P3,C3), dist(P3,C4), dist(P3,G), dist(P3,M)],
                 [dist(P4,P1), dist(P4,P2), dist(P4,P3), dist(P4,P4), dist(P4,P5), dist(P4,P6), dist(P4,P7), dist(P4,P8), dist(P4,P9), dist(P4,P10), dist(P4,P11), dist(P4,P12), dist(P4,C1), dist(P4,C2), dist(P4,C3), dist(P4,C4), dist(P4,G), dist(P4,M)],
                 [dist(P5,P1), dist(P5,P2), dist(P5,P3), dist(P5,P4), dist(P5,P5), dist(P5,P6), dist(P5,P7), dist(P5,P8), dist(P5,P9), dist(P5,P10), dist(P5,P11), dist(P5,P12), dist(P5,C1), dist(P5,C2), dist(P5,C3), dist(P5,C4), dist(P5,G), dist(P5,M)],
                 [dist(P6,P1), dist(P6,P2), dist(P6,P3), dist(P6,P4), dist(P6,P5), dist(P6,P6), dist(P6,P7), dist(P6,P8), dist(P6,P9), dist(P6,P10), dist(P6,P11), dist(P6,P12), dist(P6,C1), dist(P6,C2), dist(P6,C3), dist(P6,C4), dist(P6,G), dist(P6,M)],
                 [dist(P7,P1), dist(P7,P2), dist(P7,P3), dist(P7,P4), dist(P7,P5), dist(P7,P6), dist(P7,P7), dist(P7,P8), dist(P7,P9), dist(P7,P10), dist(P7,P11), dist(P7,P12), dist(P7,C1), dist(P7,C2), dist(P7,C3), dist(P7,C4), dist(P7,G), dist(P7,M)],
                 [dist(P8,P1), dist(P8,P2), dist(P8,P3), dist(P8,P4), dist(P8,P5), dist(P8,P6), dist(P8,P7), dist(P8,P8), dist(P8,P9), dist(P8,P10), dist(P8,P11), dist(P8,P12), dist(P8,C1), dist(P8,C2), dist(P8,C3), dist(P8,C4), dist(P8,G), dist(P8,M)],
                 [dist(P9,P1), dist(P9,P2), dist(P9,P3), dist(P9,P4), dist(P9,P5), dist(P9,P6), dist(P9,P7), dist(P9,P8), dist(P9,P9), dist(P9,P10), dist(P9,P11), dist(P9,P12), dist(P9,C1), dist(P9,C2), dist(P9,C3), dist(P9,C4), dist(P9,G), dist(P9,M)],
                 [dist(P10,P1), dist(P10,P2), dist(P10,P3), dist(P10,P4), dist(P10,P5), dist(P10,P6), dist(P10,P7), dist(P10,P8), dist(P10,P9), dist(P10,P10), dist(P10,P11), dist(P10,P12), dist(P10,C1), dist(P10,C2), dist(P10,C3), dist(P10,C4), dist(P10,G), dist(P10,M)],
                 [dist(P11,P1), dist(P11,P2), dist(P11,P3), dist(P11,P4), dist(P11,P5), dist(P11,P6), dist(P11,P7), dist(P11,P8), dist(P11,P9), dist(P11,P10), dist(P11,P11), dist(P11,P12), dist(P11,C1), dist(P11,C2), dist(P11,C3), dist(P11,C4), dist(P11,G), dist(P11,M)],
                 [dist(P12,P1), dist(P12,P2), dist(P12,P3), dist(P12,P4), dist(P12,P5), dist(P12,P6), dist(P12,P7), dist(P12,P8), dist(P12,P9), dist(P12,P10), dist(P12,P11), dist(P12,P12), dist(P12,C1), dist(P12,C2), dist(P12,C3), dist(P12,C4), dist(P12,G), dist(P12,M)],
                 [dist(C1,P1), dist(C1,P2), dist(C1,P3), dist(C1,P4), dist(C1,P5), dist(C1,P6), dist(C1,P7), dist(C1,P8), dist(C1,P9), dist(C1,P10), dist(C1,P11), dist(C1,P12), dist(C1,C1), dist(C1,C2), dist(C1,C3), dist(C1,C4), dist(C1,G), dist(C1,M)],
                 [dist(C2,P1), dist(C2,P2), dist(C2,P3), dist(C2,P4), dist(C2,P5), dist(C2,P6), dist(C2,P7), dist(C2,P8), dist(C2,P9), dist(C2,P10), dist(C2,P11), dist(C2,P12), dist(C2,C1), dist(C2,C2), dist(C2,C3), dist(C2,C4), dist(C2,G), dist(C2,M)],
                 [dist(C3,P1), dist(C3,P2), dist(C3,P3), dist(C3,P4), dist(C3,P5), dist(C3,P6), dist(C3,P7), dist(C3,P8), dist(C3,P9), dist(C3,P10), dist(C3,P11), dist(C3,P12), dist(C3,C1), dist(C3,C2), dist(C3,C3), dist(C3,C4), dist(C3,G), dist(C3,M)],
                 [dist(C4,P1), dist(C4,P2), dist(C4,P3), dist(C4,P4), dist(C4,P5), dist(C4,P6), dist(C4,P7), dist(C4,P8), dist(C4,P9), dist(C4,P10), dist(C4,P11), dist(C4,P12), dist(C4,C1), dist(C4,C2), dist(C4,C3), dist(C4,C4), dist(C4,G), dist(C4,M)],
                 [dist(G,P1), dist(G,P2), dist(G,P3), dist(G,P4), dist(G,P5), dist(G,P6), dist(G,P7), dist(G,P8), dist(G,P9), dist(G,P10), dist(G,P11), dist(G,P12), dist(G,C1), dist(G,C2), dist(G,C3), dist(G,C4), dist(G,G), dist(G,M)],
                 [dist(M,P1), dist(M,P2), dist(M,P3), dist(M,P4), dist(M,P5), dist(M,P6), dist(M,P7), dist(M,P8), dist(M,P9), dist(M,P10), dist(M,P11), dist(M,P12), dist(M,C1), dist(M,C2), dist(M,C3), dist(M,C4), dist(M,G), dist(M,M)]
                ]
        
        
        region_dict = {
            'initial': 'p1',
            'type': 'MotionTS',
            'regions':{
                'p1' : {'label':'p1', 'pose': P1, 'weights':weights[0]},
                'p2' : {'label':'p2', 'pose': P2, 'weights':weights[1]},
                'p3': {'label':'p3','pose': P3, 'weights':weights[2]},
                'p4': {'label':'p4','pose': P4, 'weights':weights[3]},
                'p5': {'label':'p5','pose': P5, 'weights':weights[4]},
                'p6': {'label':'p6','pose': P6, 'weights':weights[5]},
                'p7': {'label':'p7','pose': P7, 'weights':weights[6]},
                'p8': {'label':'p8','pose': P8, 'weights':weights[7]},
                'p9': {'label':'p9','pose': P9, 'weights':weights[8]},
                'p10': {'label':'p10','pose': P10, 'weights':weights[9]},
                'p11': {'label':'p11','pose': P11, 'weights':weights[10]},
                'p12': {'label':'p12','pose': P12, 'weights':weights[11]},
                'c1': {'label':'c1','pose': C1, 'weights':weights[12]},
                'c2': {'label':'c2','pose': C2, 'weights':weights[13]},
                'c3': {'label':'c3','pose': C3, 'weights':weights[14]},
                'c4': {'label':'c4','pose': C4, 'weights':weights[15]},
                'g': {'label':'g','pose': G, 'weights':weights[16]},
                'm': {'label':'m','pose': G, 'weights':weights[17]},
            }
        }
        # action_dict = {act_name: {cost, guard_formula, label, dependendy : {}, type}} 
        action_dict = {
            'type': 'ActionModel',
            'initial':'free', #free=none
            'actions':{
                'check_connection': {'weight':7, 'guard':'c1 || c2', 'label': 'check_connection', "dependency": {'h_check_connection':['c2', 'c4']}, 'type': 'collaborative'},
                'group': {'weight':4, 'guard':'g', 'label': 'group', "dependency": {'h_group':['g']}, 'type': 'collaborative'},
                'remove_object': {'weight':50, 'guard':'m', 'label': 'remove_object', "dependency": {'h_remove_object':['m']}, 'type': 'collaborative'},
                'h_check_connection': {'weight':5, 'guard':'c3 || c4', 'label': 'h_check_connection', "dependency": {}, 'type': 'assisting'},
                'h_group': {'weight':5, 'guard':'g', 'label': 'h_group', "dependency": {}, 'type': 'assisting'},
                'patrol': {'weight':6, 'guard':'p1 || p2 || p3 || p4 || p5 || p6 || p7 || p8 || p9 || p10 || p11 || p12', 'label': 'patrol', "dependency": {}, 'type': 'local'},
                'wait' : {"weight":20, "guard":'1', "label": 'wait', "dependency":{}, "type": 'local'},
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
print("Please enter robot type (turtlebot or rosie) ")
prompt = '> '
robot_type = input(prompt)
output_standard(file_name, robot_type)














