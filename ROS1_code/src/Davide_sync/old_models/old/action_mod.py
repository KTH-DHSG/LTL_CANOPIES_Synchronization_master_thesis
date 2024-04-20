from ltl_automaton_planner.boolean_formulas.parser import parse as parse_guard

class ActionModel(object):    
    # action_dict = {act_name: (cost, guard_formula, label, dependendy={action: set(regions)}, category)}
    def __init__(self, action_dict):
        self.raw = action_dict
        self.action = dict()
        for act_name, attrib in action_dict.items():            
            cost = attrib[0]
            # convert the guard formula to a guard expression
            guard_formula = attrib[1]
            guard_expr = parse_guard(guard_formula)
            action = attrib[2] #tecnically label but to adapt to planner 
            dependency = attrib[3]
            category = attrib[4]           
            # add the action to the action dictionary
            self.action[act_name] = (cost, guard_expr, action, dependency, category)
        # add a 'None' action
        self.action['None'] = (1, parse_guard('1'), set(), {}, 'local') 
    
    # return the allowed actions for a given node label i.e. region of the motion FTS
    # works with single labels or sets of labels independently
    def allowed_actions(self, ts_node_label):
        allow_action = set()
        for act_name, attrib in self.action.items():
            if (attrib[1].check(ts_node_label)):
                allow_action.add(act_name)
        return allow_action
    
        
class ActionModel2(object):    
    # action_dict = {act_name: (cost, guard_formula, label, dependendy={action: set(regions)}, category)}
    def __init__(self, action_dict):
        self.action = action_dict
        # add a 'None' action
        #FIXMED: NO LABEL FOR THE ACTION so not sure this respects meng but may not work
        self.action['None'] = {"weight":1, "guard":'1', "label": '', "dependency":{}, "type": 'local'}
    
    # return the allowed actions for a given node label i.e. region of the motion FTS
    # works with single labels or sets of labels independently
    def allowed_actions(self, ts_node_label):
        allow_action = set()
        for act_name, attrib in self.action.items():
            if (parse_guard(attrib["guard"]).check(ts_node_label)):
                allow_action.add(act_name)
        return allow_action