from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner
from networkx import shortest_path

from math import ceil, floor


# residential regions
R1 = (15, 45, 3)
R2 = (5, 30, 3)
R3 = (15, 10, 3)
R4 = (45, 10, 3)
R5 = (55, 30, 3)
R6 = (45, 45, 3)
R_dict = {
    R1: set(['r1']),
    R2: set(['r2']),    
    R3: set(['r3']),
    R4: set(['r4']),
    R5: set(['r5']),
    R6: set(['r6']),    
}
R_symbols = set(['r0','r1','r2','r3','r4','r5','r6'])

COST = 2 #s
dep = dict()
init_pose = []
SPEED = []
LOCAL = {'r': COST, 'p21': COST, 'd21': COST,
         'p31': COST, 'd31': COST,'p41': COST, 'd41': COST,
         'None': 1,}


#============================================================
# UAV, group1, g1 = {a11, a12, a13, a14}
g1_base = (6, 6, 1)
g1_speed = [10, 11, 12, 13, 14] #m/s
init_pose.extend([[6,6],[6,6],[6,6],[6,6],[6,6]])
SPEED.extend(g1_speed)
# agent a11
#============================== motion
a11_init_pose = g1_base
a11_node_dict = dict(R_dict)
a11_node_dict[g1_base] = set(['r0'])
a11_symbols = R_symbols
a11_motion = MotionFts(a11_node_dict, a11_symbols, 'a11_FTS')
a11_motion.set_initial(a11_init_pose)
a11_motion.add_full_edges(unit_cost=1.0/g1_speed[0])
#============================== action
a11_action_dict = {
    'r': (COST, '1', set(['r'])),
    'c': (2*COST, '1', set(['c'])),
    'hca': (2*COST, '1', set(['hca'])),
    'hcb': (2*COST, '1', set(['hcb'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),    
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),  
    
}
dep['c'] = set(['hca', 'hcb'])
a11_action = ActionModel(a11_action_dict)
#a11_hard_task = '(<>(r1 && r)) && (<>(r2 && r)) && (<>(r4 && c)) && ([]<> r0)'
a11_hard_task = '(<>(r5 && r)) && (<>(r2 && r)) && (<>(r4 && c)) && ([]<> r0)'
a11_soft_task = None
a11_model = MotActModel(a11_motion, a11_action)
a11_planner = ltl_planner(a11_model, a11_hard_task, a11_soft_task)


# agent a12
#============================== motion
a12_init_pose = g1_base
a12_node_dict = dict(R_dict)
a12_node_dict[g1_base] = set(['r0'])
a12_symbols = R_symbols
a12_motion = MotionFts(a12_node_dict, a12_symbols, 'a12_FTS')
a12_motion.set_initial(a12_init_pose)
a12_motion.add_full_edges(unit_cost=1.0/g1_speed[1])
#============================== action
a12_action_dict = {
    'r': (COST, '1', set(['r'])),
    'c': (2*COST, '1', set(['c'])),
    'hca': (2*COST, '1', set(['hca'])),
    'hcb': (2*COST, '1', set(['hcb'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),    
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),  
}
dep['c'] = set(['hca', 'hcb'])
a12_action = ActionModel(a12_action_dict)
#a12_hard_task = '(<>(r5 && r)) && (<>(r2 && r)) && (<>(r3 && r))  && ([]<> r0)'
a12_hard_task = '(<>(r1 && r)) && (<>(r2 && r)) && (<>(r3 && r))  && ([]<> r0)'
a12_soft_task = None
a12_model = MotActModel(a12_motion, a12_action)
a12_planner = ltl_planner(a12_model, a12_hard_task, a12_soft_task)

#### a11, a12 switch r1&r , r5&r



# agent a13
#============================== motion
a13_init_pose = g1_base
a13_node_dict = dict(R_dict)
a13_node_dict[g1_base] = set(['r0'])
a13_symbols = R_symbols
a13_motion = MotionFts(a13_node_dict, a13_symbols, 'a13_FTS')
a13_motion.set_initial(a13_init_pose)
a13_motion.add_full_edges(unit_cost=1.0/g1_speed[2])
#============================== action
a13_action_dict = {
    'r': (COST, '1', set(['r'])),
    'c': (2*COST, '1', set(['c'])),
    'hca': (2*COST, '1', set(['hca'])),
    'hcb': (2*COST, '1', set(['hcb'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),    
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),      
}

a13_action = ActionModel(a13_action_dict)
#a13_hard_task = '(<>(r3 && r)) && (<>(r6 && c)) && (<>(r4 && r))  && ([]<> r0)'
a13_hard_task = '(<>(r3 && r)) && (<>(r5 && r)) && (<>(r4 && r))  && ([]<> r0)'
a13_soft_task = None
a13_model = MotActModel(a13_motion, a13_action)
a13_planner = ltl_planner(a13_model, a13_hard_task, a13_soft_task)


# agent a14
#============================== motion
a14_init_pose = g1_base
a14_node_dict = dict(R_dict)
a14_node_dict[g1_base] = set(['r0'])
a14_symbols = R_symbols
a14_motion = MotionFts(a14_node_dict, a14_symbols, 'a14_FTS')
a14_motion.set_initial(a14_init_pose)
a14_motion.add_full_edges(unit_cost=1.0/g1_speed[3])
#============================== action
a14_action_dict = {
    'r': (COST, '1', set(['r'])),
    'c': (2*COST, '1', set(['c'])),
    'hca': (2*COST, '1', set(['hca'])),
    'hcb': (2*COST, '1', set(['hcb'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),    
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),        
}

a14_action = ActionModel(a14_action_dict)
#a14_hard_task = '(<>(r4 && r)) && (<>(r1 && r)) && (<>(r5 && r))  && ([]<> r0)'
a14_hard_task = '(<>(r4 && r)) && (<>(r1 && r)) && (<>(r6 && c))  && ([]<> r0)'
a14_soft_task = None
a14_model = MotActModel(a14_motion, a14_action)
a14_planner = ltl_planner(a14_model, a14_hard_task, a14_soft_task)


#### a13, a14 switch r6&c , r5&r


# agent a15
#============================== motion
a15_init_pose = g1_base
a15_node_dict = dict(R_dict)
a15_node_dict[g1_base] = set(['r0'])
a15_symbols = R_symbols
a15_motion = MotionFts(a15_node_dict, a15_symbols, 'a15_FTS')
a15_motion.set_initial(a15_init_pose)
a15_motion.add_full_edges(unit_cost=1.0/g1_speed[4])
#============================== action
a15_action_dict = {
    'r': (COST, '1', set(['r'])),
    'c': (2*COST, '1', set(['c'])),
    'hca': (2*COST, '1', set(['hca'])),
    'hcb': (2*COST, '1', set(['hcb'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),    
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),      
}

a15_action = ActionModel(a15_action_dict)
a15_hard_task = '(<>(r2 && c)) && (<>(r5 && r)) && (<>(r6 && r))  && ([]<> r0)'
a15_soft_task = None
a15_model = MotActModel(a15_motion, a15_action)
a15_planner = ltl_planner(a15_model, a15_hard_task, a15_soft_task)






#============================================================
# UGV, group2, g2={a21, a22, a23, a24}
g2_base = (54, 6, 1)
g2_speed = [6,7,8,9,10] #m/s
g2_symbols = R_symbols.union(set(['o21', 'o22', 'o23']))
init_pose.extend([[54,6],[54,6],[54,6],[54,6],[54,6]])
SPEED.extend(g2_speed)
# storage
S1 = (30, 50, 3)
S2 = (30, 30, 3)
S3 = (30, 10, 3)
Sg2_dict = {
    S1: set(['o21']),
    S2: set(['o22']),
    S3: set(['o23']),    
}

# agent a21
#============================== motion
a21_init_pose = g2_base
a21_node_dict = dict(R_dict)
a21_node_dict[g2_base] = set(['r0'])
a21_node_dict.update(Sg2_dict)
a21_symbols = g2_symbols
a21_motion = MotionFts(a21_node_dict, a21_symbols, 'a21_FTS')
a21_motion.set_initial(a21_init_pose)
a21_motion.add_full_edges(unit_cost=1.0/g2_speed[0])
#============================== action
a21_action_dict = {
    'p21': (2*COST, 'o21', set(['p21'])),
    'd21': (COST, '1', set(['d21'])),
    'p22': (2*COST, 'o22', set(['p22'])),
    'd22': (COST, '1', set(['d22'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'p23': (2*COST, 'o23', set(['p23'])),
    'd23': (COST, '1', set(['d23'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),    
}
dep['p22'] = set(['hp22'])
dep['d22'] = set(['hd22'])
dep['p23'] = set(['hpa33', 'hpb43'])
dep['d23'] = set(['hda33', 'hdb43'])
a21_action = ActionModel(a21_action_dict)
#a21_hard_task = '(<>( p21 && <> (r1 && d21))) && (<>( p22 && <> (r2 && d22))) && (<>( p23 && <> (r3 && d23))) && ([]<> r0)'
a21_hard_task = '(<>( p21 && <> (r4 && d21))) && (<>( p21 && <> (r6 && d21))) && (<>( p21 && <> (r1 && d21))) && (<>( p22 && <> (r2 && d22))) && (<>( p23 && <> (r3 && d23))) && ([]<> r0)'
a21_soft_task = None
a21_model = MotActModel(a21_motion, a21_action)
a21_planner = ltl_planner(a21_model, a21_hard_task, a21_soft_task)


# agent a22
#============================== motion
a22_init_pose = g2_base
a22_node_dict = dict(R_dict)
a22_node_dict[g2_base] = set(['r0'])
a22_node_dict.update(Sg2_dict)
a22_symbols = g2_symbols
a22_motion = MotionFts(a22_node_dict, a22_symbols, 'a22_FTS')
a22_motion.set_initial(a22_init_pose)
a22_motion.add_full_edges(unit_cost=1.0/g2_speed[1])
#============================== action
a22_action_dict = {
    'p21': (2*COST, 'o21', set(['p21'])),
    'd21': (COST, '1', set(['d21'])),
    'p22': (2*COST, 'o22', set(['p22'])),
    'd22': (COST, '1', set(['d22'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'p23': (2*COST, 'o23', set(['p23'])),
    'd23': (COST, '1', set(['d23'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),    
}
dep['p22'] = set(['hp22'])
dep['d22'] = set(['hd22'])
dep['p23'] = set(['hpa33', 'hpb43'])
dep['d23'] = set(['hda33', 'hdb43'])
a22_action = ActionModel(a22_action_dict)
a22_hard_task = '(<>( p21 && <> (r3 && d21))) && (<>( p21 && <> (r2 && d21))) && (<>( p22 && <> (r4 && d22))) && ([]<> r0)'
a22_soft_task = None
a22_model = MotActModel(a22_motion, a22_action)
a22_planner = ltl_planner(a22_model, a22_hard_task, a22_soft_task)


# agent a23
#============================== motion
a23_init_pose = g2_base
a23_node_dict = dict(R_dict)
a23_node_dict[g2_base] = set(['r0'])
a23_node_dict.update(Sg2_dict)
a23_symbols = g2_symbols
a23_motion = MotionFts(a23_node_dict, a23_symbols, 'a23_FTS')
a23_motion.set_initial(a23_init_pose)
a23_motion.add_full_edges(unit_cost=1.0/g2_speed[2])
#============================== action
a23_action_dict = {
    'p21': (2*COST, 'o21', set(['p21'])),
    'd21': (COST, '1', set(['d21'])),
    'p22': (2*COST, 'o22', set(['p22'])),
    'd22': (COST, '1', set(['d22'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'p23': (2*COST, 'o23', set(['p23'])),
    'd23': (COST, '1', set(['d23'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),    
}
dep['p22'] = set(['hp22'])
dep['d22'] = set(['hd22'])
dep['p23'] = set(['hpa33', 'hpb43'])
dep['d23'] = set(['hda33', 'hdb43'])
a23_action = ActionModel(a23_action_dict)
#a23_hard_task = '(<>( p22 && <> (r4 && d22))) && (<>( p22 && <> (r5 && d22))) && (<>( p23 && <> (r3 && d23))) && ([]<> r0)'
a23_hard_task = '(<>( p22 && <> (r5 && d22))) && (<>( p23 && <> (r3 && d23))) && ([]<> r0)'
a23_soft_task = None
a23_model = MotActModel(a23_motion, a23_action)
a23_planner = ltl_planner(a23_model, a23_hard_task, a23_soft_task)



# agent a23
#============================== motion
a24_init_pose = g2_base
a24_node_dict = dict(R_dict)
a24_node_dict[g2_base] = set(['r0'])
a24_node_dict.update(Sg2_dict)
a24_symbols = g2_symbols
a24_motion = MotionFts(a24_node_dict, a24_symbols, 'a24_FTS')
a24_motion.set_initial(a24_init_pose)
a24_motion.add_full_edges(unit_cost=1.0/g2_speed[3])
#============================== action
a24_action_dict = {
    'p21': (2*COST, 'o21', set(['p21'])),
    'd21': (COST, '1', set(['d21'])),
    'p22': (2*COST, 'o22', set(['p22'])),
    'd22': (COST, '1', set(['d22'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'p23': (2*COST, 'o23', set(['p23'])),
    'd23': (COST, '1', set(['d23'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),    
}
dep['p22'] = set(['hp22'])
dep['d22'] = set(['hd22'])
dep['p23'] = set(['hpa33', 'hpb43'])
dep['d23'] = set(['hda33', 'hdb43'])
a24_action = ActionModel(a24_action_dict)
#a24_hard_task = '(<>( p21 && <> (r3 && d21))) && (<>( p23 && <> (r4 && d23))) && (<>( p23 && <> (r5 && d23))) && ([]<> r0)'
a24_hard_task = '(<>( p21 && <> (r3 && d21))) && (<>( p23 && <> (r5 && d23))) && ([]<> r0)'
a24_soft_task = None
a24_model = MotActModel(a24_motion, a24_action)
a24_planner = ltl_planner(a24_model, a24_hard_task, a24_soft_task)


# agent a25
#============================== motion
a25_init_pose = g2_base
a25_node_dict = dict(R_dict)
a25_node_dict[g2_base] = set(['r0'])
a25_node_dict.update(Sg2_dict)
a25_symbols = g2_symbols
a25_motion = MotionFts(a25_node_dict, a25_symbols, 'a25_FTS')
a25_motion.set_initial(a25_init_pose)
a25_motion.add_full_edges(unit_cost=1.0/g2_speed[4])
#============================== action
a25_action_dict = {
    'p21': (2*COST, 'o21', set(['p21'])),
    'd21': (COST, '1', set(['d21'])),
    'p22': (2*COST, 'o22', set(['p22'])),
    'd22': (COST, '1', set(['d22'])),
    'hp22': (2*COST, '1', set(['hp22'])),
    'hd22': (COST, '1', set(['hd22'])),
    'p23': (2*COST, 'o23', set(['p23'])),
    'd23': (COST, '1', set(['d23'])),
    'hpa23': (2*COST, '1', set(['hpa23'])),
    'hda23': (COST, '1', set(['hda23'])),
    'hpb23': (2*COST, '1', set(['hpb23'])),
    'hdb23': (COST, '1', set(['hdb23'])),    
}
dep['p22'] = set(['hp22'])
dep['d22'] = set(['hd22'])
dep['p23'] = set(['hpa33', 'hpb43'])
dep['d23'] = set(['hda33', 'hdb43'])
a25_action = ActionModel(a25_action_dict)
#a25_hard_task = '(<>( p21 && <> (r4 && d21))) && (<>( p22 && <> (r5 && d22))) && (<>( p21 && <> (r6 && d21))) && ([]<> r0)'
a25_hard_task = '(<>( p22 && <> (r5 && d22))) && ([]<> r0)'
a25_soft_task = None
a25_model = MotActModel(a25_motion, a25_action)
a25_planner = ltl_planner(a25_model, a25_hard_task, a25_soft_task)

####### a21 takes p21, d21 at r4, r6

#============================================================
# UGV, group3, g3={a31, a32, a33, a34}
g3_base = (54, 54, 1)
g3_speed = [6,7,8,9,10] #m/s
g3_symbols = R_symbols.union(set(['o31', 'o32', 'o33']))
init_pose.extend([[54, 54],[54,54],[54,54],[54,54],[54,54]])
SPEED.extend(g3_speed)
# storage
Sg3_dict = {
    S1: set(['o31']),
    S2: set(['o32']),
    S3: set(['o33']),    
}

# agent a31
#============================== motion
a31_init_pose = g3_base
a31_node_dict = dict(R_dict)
a31_node_dict[g3_base] = set(['r0'])
a31_node_dict.update(Sg3_dict)
a31_symbols = g3_symbols
a31_motion = MotionFts(a31_node_dict, a31_symbols, 'a31_FTS')
a31_motion.set_initial(a31_init_pose)
a31_motion.add_full_edges(unit_cost=1.0/g3_speed[0])
#============================== action
a31_action_dict = {
    'p31': (2*COST, 'o31', set(['p31'])),
    'd31': (COST, '1', set(['d31'])),
    'p32': (2*COST, 'o32', set(['p32'])),
    'd32': (COST, '1', set(['d32'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'p33': (2*COST, 'o33', set(['p33'])),
    'd33': (COST, '1', set(['d33'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),    
}
dep['p32'] = set(['hp32'])
dep['d32'] = set(['hd32'])
dep['p33'] = set(['hpa43', 'hpb23'])
dep['d33'] = set(['hda43', 'hdb23'])
a31_action = ActionModel(a31_action_dict)
a31_hard_task = '(<>( p31 && <> (r6 && d31))) && (<>( p31 && <> (r5 && d31))) && (<>( p32 && <> (r1 && d32))) && ([]<> r0)'
#a31_hard_task = '(<>( p31 && <> (r3 && d31))) && (<>( p31 && <> (r2 && d31))) && (<>( p31 && <> (r6 && d31))) && (<>( p31 && <> (r5 && d31))) && (<>( p32 && <> (r1 && d32))) && ([]<> r0)'
a31_soft_task = None
a31_model = MotActModel(a31_motion, a31_action)
a31_planner = ltl_planner(a31_model, a31_hard_task, a31_soft_task)


# agent a32
#============================== motion
a32_init_pose = g3_base
a32_node_dict = dict(R_dict)
a32_node_dict[g3_base] = set(['r0'])
a32_node_dict.update(Sg3_dict)
a32_symbols = g3_symbols
a32_motion = MotionFts(a32_node_dict, a32_symbols, 'a32_FTS')
a32_motion.set_initial(a32_init_pose)
a32_motion.add_full_edges(unit_cost=1.0/g3_speed[1])
#============================== action
a32_action_dict = {
    'p31': (2*COST, 'o31', set(['p31'])),
    'd31': (COST, '1', set(['d31'])),
    'p32': (2*COST, 'o32', set(['p32'])),
    'd32': (COST, '1', set(['d32'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'p33': (2*COST, 'o33', set(['p33'])),
    'd33': (COST, '1', set(['d33'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),    
}
dep['p32'] = set(['hp32'])
dep['d32'] = set(['hd32'])
dep['p33'] = set(['hpa43', 'hpb23'])
dep['d33'] = set(['hda43', 'hdb23'])
a32_action = ActionModel(a32_action_dict)
#a32_hard_task = ' (<>( p32 && <> (r1 && d32))) && (<>( p32 && <> (r2 && d32))) && (<>( p31 && <> (r6 && d31)))&& ([]<> r0)'
#a32_hard_task = '  (<>( p32 && <> (r2 && d32))) && (<>( p31 && <> (r6 && d31)))&& ([]<> r0)'
# 
a32_hard_task = '(<>( p33 && <> (r4 && d33))) && (<>( p31 && <> (r6 && d31)))&& ([]<> r0)'
a32_soft_task = None
a32_model = MotActModel(a32_motion, a32_action)
a32_planner = ltl_planner(a32_model, a32_hard_task, a32_soft_task)



# agent a33
#============================== motion
a33_init_pose = g3_base
a33_node_dict = dict(R_dict)
a33_node_dict[g3_base] = set(['r0'])
a33_node_dict.update(Sg3_dict)
a33_symbols = g3_symbols
a33_motion = MotionFts(a33_node_dict, a33_symbols, 'a33_FTS')
a33_motion.set_initial(a33_init_pose)
a33_motion.add_full_edges(unit_cost=1.0/g3_speed[2])
#============================== action
a33_action_dict = {
    'p31': (2*COST, 'o31', set(['p31'])),
    'd31': (COST, '1', set(['d31'])),
    'p32': (2*COST, 'o32', set(['p32'])),
    'd32': (COST, '1', set(['d32'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'p33': (2*COST, 'o33', set(['p33'])),
    'd33': (COST, '1', set(['d33'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),    
}
dep['p32'] = set(['hp32'])
dep['d32'] = set(['hd32'])
dep['p33'] = set(['hpa43', 'hpb23'])
dep['d33'] = set(['hda43', 'hdb23'])
a33_action = ActionModel(a33_action_dict)
#a33_hard_task = '(<>( p31 && <> (r2 && d31))) && (<>( p33 && <> (r2 && d33))) && (<>( p32 && <> (r3 && d32))) && ([]<> r0)'
#a33_hard_task = '(<>( p33 && <> (r4 && d33))) && (<>( p33 && <> (r2 && d33))) && (<>( p32 && <> (r3 && d32))) && ([]<> r0)'
a33_hard_task = '(<>( p33 && <> (r2 && d33))) && (<>( p32 && <> (r3 && d32))) && ([]<> r0)'
a33_soft_task = None
a33_model = MotActModel(a33_motion, a33_action)
a33_planner = ltl_planner(a33_model, a33_hard_task, a33_soft_task)

#######a31, a33 takes (<>( p31 && <> (r2 && d31))) && 

# agent a34
#============================== motion
a34_init_pose = g3_base
a34_node_dict = dict(R_dict)
a34_node_dict[g3_base] = set(['r0'])
a34_node_dict.update(Sg3_dict)
a34_symbols = g3_symbols
a34_motion = MotionFts(a34_node_dict, a34_symbols, 'a34_FTS')
a34_motion.set_initial(a34_init_pose)
a34_motion.add_full_edges(unit_cost=1.0/g3_speed[3])
#============================== action
a34_action_dict = {
    'p31': (2*COST, 'o31', set(['p31'])),
    'd31': (COST, '1', set(['d31'])),
    'p32': (2*COST, 'o32', set(['p32'])),
    'd32': (COST, '1', set(['d32'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'p33': (2*COST, 'o33', set(['p33'])),
    'd33': (COST, '1', set(['d33'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),    
}
dep['p32'] = set(['hp32'])
dep['d32'] = set(['hd32'])
dep['p33'] = set(['hpa43', 'hpb23'])
dep['d33'] = set(['hda43', 'hdb23'])
a34_action = ActionModel(a34_action_dict)
a34_hard_task = '(<>( p32 && <> (r2 && d32))) && (<>( p31 && <> (r4 && d31))) && ([]<> r0)'
a34_soft_task = None
a34_model = MotActModel(a34_motion, a34_action)
a34_planner = ltl_planner(a34_model, a34_hard_task, a34_soft_task)


####### a31, a34 takes (<>( p31 && <> (r3 && d31))) && 

# agent a35
#============================== motion
a35_init_pose = g3_base
a35_node_dict = dict(R_dict)
a35_node_dict[g3_base] = set(['r0'])
a35_node_dict.update(Sg3_dict)
a35_symbols = g3_symbols
a35_motion = MotionFts(a35_node_dict, a35_symbols, 'a35_FTS')
a35_motion.set_initial(a35_init_pose)
a35_motion.add_full_edges(unit_cost=1.0/g3_speed[4])
#============================== action
a35_action_dict = {
    'p31': (2*COST, 'o31', set(['p31'])),
    'd31': (COST, '1', set(['d31'])),
    'p32': (2*COST, 'o32', set(['p32'])),
    'd32': (COST, '1', set(['d32'])),
    'hp32': (2*COST, '1', set(['hp32'])),
    'hd32': (COST, '1', set(['hd32'])),
    'p33': (2*COST, 'o33', set(['p33'])),
    'd33': (COST, '1', set(['d33'])),
    'hpa33': (2*COST, '1', set(['hpa33'])),
    'hda33': (COST, '1', set(['hda33'])),
    'hpb33': (2*COST, '1', set(['hpb33'])),
    'hdb33': (COST, '1', set(['hdb33'])),    
}
dep['p32'] = set(['hp32'])
dep['d32'] = set(['hd32'])
dep['p33'] = set(['hpa43', 'hpb23'])
dep['d33'] = set(['hda43', 'hdb23'])
a35_action = ActionModel(a35_action_dict)
#a35_hard_task = ' (<>( p33 && <> (r4 && d33))) && (<>( p32 && <> (r5 && d32))) && ([]<> r0)'
a35_hard_task = ' (<>( p32 && <> (r2 && d32))) && (<>( p32 && <> (r5 && d32))) && ([]<> r0)'
a35_soft_task = None
a35_model = MotActModel(a35_motion, a35_action)
a35_planner = ltl_planner(a35_model, a35_hard_task, a35_soft_task)

####a35, a32, (<>( p33 && <> (r4 && d33))), (<>( p32 && <> (r2 && d32)))


#============================================================
# UGV, group4, g4={a41, a42, a43, a44, a45}
g4_base = (6, 54, 1)
g4_speed = [4,5,6,7,8] #m/s
g4_symbols = R_symbols.union(set(['o41', 'o42', 'o43']))
init_pose.extend([[6, 54],[6,54],[6,54],[6,54],[6,54]])
SPEED.extend(g4_speed)
# storage
Sg4_dict = {
    S1: set(['o41']),
    S2: set(['o42']),
    S3: set(['o43']),    
}

# agent a41
#============================== motion
a41_init_pose = g4_base
a41_node_dict = dict(R_dict)
a41_node_dict[g4_base] = set(['r0'])
a41_node_dict.update(Sg4_dict)
a41_symbols = g4_symbols
a41_motion = MotionFts(a41_node_dict, a41_symbols, 'a41_FTS')
a41_motion.set_initial(a41_init_pose)
a41_motion.add_full_edges(unit_cost=1.0/g4_speed[0])
#============================== action
a41_action_dict = {
    'p41': (2*COST, 'o41', set(['p41'])),
    'd41': (COST, '1', set(['d41'])),
    'p42': (2*COST, 'o42', set(['p42'])),
    'd42': (COST, '1', set(['d42'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'p43': (2*COST, 'o43', set(['p43'])),
    'd43': (COST, '1', set(['d43'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),    
}
dep['p42'] = set(['hp42'])
dep['d42'] = set(['hd42'])
dep['p43'] = set(['hpa23', 'hpb33'])
dep['d43'] = set(['hda23', 'hdb33'])
a41_action = ActionModel(a41_action_dict)
#a41_hard_task = '(<>( p41 && <> (r4 && d41))) && (<>( p41 && <> (r5 && d41))) && (<>( p41 && <> (r6 && d41))) && ([]<> r0)'
a41_hard_task = ' (<>( p41 && <> (r5 && d41))) && (<>( p41 && <> (r6 && d41))) && ([]<> r0)'
a41_soft_task = None
a41_model = MotActModel(a41_motion, a41_action)
a41_planner = ltl_planner(a41_model, a41_hard_task, a41_soft_task)


# agent a42
#============================== motion
a42_init_pose = g4_base
a42_node_dict = dict(R_dict)
a42_node_dict[g4_base] = set(['r0'])
a42_node_dict.update(Sg4_dict)
a42_symbols = g4_symbols
a42_motion = MotionFts(a42_node_dict, a42_symbols, 'a42_FTS')
a42_motion.set_initial(a42_init_pose)
a42_motion.add_full_edges(unit_cost=1.0/g4_speed[1])
#============================== action
a42_action_dict = {
    'p41': (2*COST, 'o41', set(['p41'])),
    'd41': (COST, '1', set(['d41'])),
    'p42': (2*COST, 'o42', set(['p42'])),
    'd42': (COST, '1', set(['d42'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'p43': (2*COST, 'o43', set(['p43'])),
    'd43': (COST, '1', set(['d43'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),    
}
dep['p42'] = set(['hp42'])
dep['d42'] = set(['hd42'])
dep['p43'] = set(['hpa23', 'hpb33'])
dep['d43'] = set(['hda23', 'hdb33'])
a42_action = ActionModel(a42_action_dict)
#a42_hard_task = ' (<>( p42 && <> (r6 && d42)))  && (<>( p43 && <> (r1 && d43))) && ([]<> r0)'
a42_hard_task = '(<>( p43 && <> (r2 && d43)))  && (<>( p43 && <> (r1 && d43))) && ([]<> r0)'
a42_soft_task = None
a42_model = MotActModel(a42_motion, a42_action)
a42_planner = ltl_planner(a42_model, a42_hard_task, a42_soft_task)


# agent a43
#============================== motion
a43_init_pose = g4_base
a43_node_dict = dict(R_dict)
a43_node_dict[g4_base] = set(['r0'])
a43_node_dict.update(Sg4_dict)
a43_symbols = g4_symbols
a43_motion = MotionFts(a43_node_dict, a43_symbols, 'a43_FTS')
a43_motion.set_initial(a43_init_pose)
a43_motion.add_full_edges(unit_cost=1.0/g4_speed[2])
#============================== action
a43_action_dict = {
    'p41': (2*COST, 'o41', set(['p41'])),
    'd41': (COST, '1', set(['d41'])),
    'p42': (2*COST, 'o42', set(['p42'])),
    'd42': (COST, '1', set(['d42'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'p43': (2*COST, 'o43', set(['p43'])),
    'd43': (COST, '1', set(['d43'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),    
}
dep['p42'] = set(['hp42'])
dep['d42'] = set(['hd42'])
dep['p43'] = set(['hpa23', 'hpb33'])
dep['d43'] = set(['hda23', 'hdb33'])
a43_action = ActionModel(a43_action_dict)
#a43_hard_task = '(<>( p42 && <> (r1 && d42))) && (<>( p43 && <> (r2 && d43)))  && ([]<> r0)'
a43_hard_task = ' (<>( p42 && <> (r6 && d42)))  && (<>( p42 && <> (r1 && d42))) &&  ([]<> r0)'
a43_soft_task = None
a43_model = MotActModel(a43_motion, a43_action)
a43_planner = ltl_planner(a43_model, a43_hard_task, a43_soft_task)

######## a43, a42  (<>( p42 && <> (r6 && d42)))  && and (<>( p43 && <> (r2 && d43)))  &&

# agent a44
#============================== motion
a44_init_pose = g4_base
a44_node_dict = dict(R_dict)
a44_node_dict[g4_base] = set(['r0'])
a44_node_dict.update(Sg4_dict)
a44_symbols = g4_symbols
a44_motion = MotionFts(a44_node_dict, a44_symbols, 'a44_FTS')
a44_motion.set_initial(a44_init_pose)
a44_motion.add_full_edges(unit_cost=1.0/g4_speed[3])
#============================== action
a44_action_dict = {
    'p41': (2*COST, 'o41', set(['p41'])),
    'd41': (COST, '1', set(['d41'])),
    'p42': (2*COST, 'o42', set(['p42'])),
    'd42': (COST, '1', set(['d42'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'p43': (2*COST, 'o43', set(['p43'])),
    'd43': (COST, '1', set(['d43'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),    
}
dep['p42'] = set(['hp42'])
dep['d42'] = set(['hd42'])
dep['p43'] = set(['hpa23', 'hpb33'])
dep['d43'] = set(['hda23', 'hdb33'])
a44_action = ActionModel(a44_action_dict)
a44_hard_task = '(<>( p41 && <> (r1 && d41))) && (<>( p41 && <> (r2 && d41))) && (<>( p41 && <> (r6 && d41))) && ([]<> r0)'
a44_soft_task = None
a44_model = MotActModel(a44_motion, a44_action)
a44_planner = ltl_planner(a44_model, a44_hard_task, a44_soft_task)


# agent a45
#============================== motion
a45_init_pose = g4_base
a45_node_dict = dict(R_dict)
a45_node_dict[g4_base] = set(['r0'])
a45_node_dict.update(Sg4_dict)
a45_symbols = g4_symbols
a45_motion = MotionFts(a45_node_dict, a45_symbols, 'a45_FTS')
a45_motion.set_initial(a45_init_pose)
a45_motion.add_full_edges(unit_cost=1.0/g4_speed[4])
#============================== action
a45_action_dict = {
    'p41': (2*COST, 'o41', set(['p41'])),
    'd41': (COST, '1', set(['d41'])),
    'p42': (2*COST, 'o42', set(['p42'])),
    'd42': (COST, '1', set(['d42'])),
    'hp42': (2*COST, '1', set(['hp42'])),
    'hd42': (COST, '1', set(['hd42'])),
    'p43': (2*COST, 'o43', set(['p43'])),
    'd43': (COST, '1', set(['d43'])),
    'hpa43': (2*COST, '1', set(['hpa43'])),
    'hda43': (COST, '1', set(['hda43'])),
    'hpb43': (2*COST, '1', set(['hpb43'])),
    'hdb43': (COST, '1', set(['hdb43'])),    
}
dep['p42'] = set(['hp42'])
dep['d42'] = set(['hd42'])
dep['p43'] = set(['hpa23', 'hpb33'])
dep['d43'] = set(['hda23', 'hdb33'])
a45_action = ActionModel(a45_action_dict)
#a45_hard_task = '(<>( p42 && <> (r1 && d42))) && (<>( p42 && <> (r2 && d42))) && (<>( p42 && <> (r5 && d42))) && ([]<> r0)'
a45_hard_task = '(<>( p42 && <> (r5 && d42))) && ([]<> r0)'
a45_soft_task = None
a45_model = MotActModel(a45_motion, a45_action)
a45_planner = ltl_planner(a45_model, a45_hard_task, a45_soft_task)



#==============================
H = 1.5
WS_node_dict = {(H*2*k, H*2*j, H): set() for k in range(0, 21) for j in range(0, 21)}

WS_symbols = set(['g1', 'g2', 'g3', 'g4', 'r1', 'r2', 'r3', 'r4', 'r5', 'r6',
                  'S1', 'S2', 'S3'])


for key, value in R_dict.iteritems():
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            if (abs(i)+abs(j)<=1):
                WS_node_dict[((floor(key[0]/(2*H))+i)*(2*H), (floor(key[1]/(2*H))+j)*(2*H), H)] = value

G = [[g1_base, 'g1'], [g2_base, 'g2'], [g3_base, 'g3'], [g4_base, 'g4']]
for g in G:
    WS_node_dict[(floor(g[0][0]/(2*H))*(2*H), floor(g[0][1]/(2*H))*(2*H), H)] = set([g[1],])

S = [[S1, 'S1'], [S2, 'S2'], [S3, 'S3']]

for s in S:
    for i in [-1, 0, 1]:
        for j in [-1, 0, 1]:
            if (abs(i)+abs(j)<=1):            
                WS_node_dict[((floor(s[0][0]/(2*H))+i)*(2*H), (floor(s[0][1]/(2*H))+j)*(2*H), H)] = set([s[1],])

Obs = [(5*2*H, 6*2*H, H), (6*2*H, 6*2*H, H), (7*2*H, 6*2*H, H), (8*2*H, 6*2*H, H), (4*2*H, 6*2*H, H), (3*2*H, 6*2*H, H), (2*2*H, 6*2*H, H),
        (5*2*H, 10*2*H, H), (6*2*H, 11*2*H, H), (5*2*H, 11*2*H, H),
       (5*2*H, 9*2*H, H), (6*2*H, 10*2*H, H), (7*2*H, 11*2*H, H), 
       (5*2*H, 18*2*H, H), (6*2*H, 18*2*H, H), (7*2*H, 18*2*H, H), (8*2*H, 18*2*H, H),
       (8*2*H, 17*2*H, H), (8*2*H, 16*2*H, H), (8*2*H, 15*2*H, H), (8*2*H, 14*2*H, H),
       (15*2*H, 7*2*H, H), (16*2*H, 7*2*H, H), (16*2*H, 8*2*H, H), (16*2*H, 9*2*H, H), (16*2*H, 10*2*H, H),(15*2*H, 10*2*H, H),
        (15*2*H, 8*2*H, H), (15*2*H, 9*2*H, H),
       (12*2*H, 16*2*H, H), (13*2*H, 17*2*H, H), (14*2*H, 18*2*H, H),       
]    

for obs in Obs:
    WS_node_dict[obs] = set(['obs'])

    
    
WS_ts = MotionFts(WS_node_dict, WS_symbols, 'WS_ts')
for n1 in WS_ts.nodes_iter():
    for n2 in WS_ts.nodes_iter():
        if ((abs(n1[0]-n2[0])**2 + abs(n1[1]-n2[1])**2)<=4*n1[2]**2):
            WS_ts.add_un_edges([[n1, n2]], unit_cost = 0.2)


def route_ws(Ws_ts, x0, goal):
    init_node = Ws_ts.closest_node(x0)
    goal_node = Ws_ts.closest_node(goal)
    # find allowed sub-graph
    allowed_nodes = dict()    
    for reg in WS_ts.nodes_iter():
        if not Ws_ts.node[reg]['label']: 
            allowed_nodes[reg] = set()
        else:
            if ((Ws_ts.node[reg]['label'] == Ws_ts.node[init_node]['label'])
                or (Ws_ts.node[reg]['label'] == Ws_ts.node[goal_node]['label'])):
                allowed_nodes[reg] = set()
    allowed_ws = MotionFts(allowed_nodes, Ws_ts.node[init_node]['label'].union(Ws_ts.node[goal_node]['label']), 'allowed_ts')
    for reg1 in allowed_ws.nodes_iter():
        for reg2 in WS_ts.neighbors(reg1):
            if reg2 in allowed_ws.nodes():
                allowed_ws.add_edge(reg1, reg2, weight=WS_ts.edge[reg1][reg2]['weight'])
    # find the shortest path
    path = shortest_path(allowed_ws, source=init_node, target=goal_node)
    return [0, path]
    

            


