import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import networkx as nx
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
from ltl_automaton_messages.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from std_msgs.msg import String, Header
import yaml



class SynchroActions(Node):
    def __init__(self):
        super().__init__('action_node')
        self.init_params()
        self.build_automaton()
        self.setup_pub_sub()


    def init_params(self):
        
        # retrieving full dictionary and retrieving motion and action dictionaries        
        motion_action_dictionary_path = self.declare_parameter('motion_action_dictionary_path', '').value
        
        with open(motion_action_dictionary_path, 'r') as file:
            motion_action_dictionary = yaml.safe_load(file)
            self.motion_dict = motion_action_dictionary['motion']
            self.action_dict = motion_action_dictionary['action']
        
        self.agent = self.get_namespace()
    
    
    def build_automaton(self):
        
        # motion and actions
        motion_ts = MotionTS(self.motion_dict)
        action_mod = ActionModel(self.action_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = MotActTS(motion_ts, action_mod)
        self.robot_model.build_full()
        
        self.current_state = list(self.robot_model.graph['initial'])[0]
        print(self.current_state)

    def setup_pub_sub(self):
        self.qos_profile = rclpy.qos.QoSProfile(depth=1, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publisher for the current state
        self.state_pub = self.create_publisher(TransitionSystemStateStamped, 'ts_state', self.qos_profile) 

        # Subscriber for the actions to be executed
        self.action_sub = self.create_subscription(String, 'next_move_cmd', self.action_callback, self.qos_profile)
    
    
    def action_callback(self, msg):
        next_state = ''
        weight = 0
        for state in self.robot_model.successors(self.current_state):
            if self.robot_model[self.current_state][state]['action'] == msg.data:
                next_state = state                
                weight = self.robot_model[self.current_state][state]['weight']
                self.current_state = state                
                break

        start_time=self.get_clock().now().to_msg().sec
        while(self.get_clock().now().to_msg().sec-start_time<weight):
            pass

        next_state_msg = TransitionSystemStateStamped()
        
        next_msg = TransitionSystemState()
        next_msg.states = [next_state[0], next_state[1]]
        next_msg.state_dimension_names = ['MotionTS', 'ActionModel']
        next_state_msg.ts_state = next_msg
        
        header_msg = Header()
        header_msg.stamp = self.get_clock().now().to_msg()
        next_state_msg.header = header_msg
        self.get_logger().info('ACTION NODE: Publishing State: %s. After Action: %s' %(str(next_state), msg.data))
        self.state_pub.publish(next_state_msg)
            














#==============================
#             Main
#==============================
def main():
    rclpy.init()   
    action_node = SynchroActions() 
    executor = MultiThreadedExecutor() 
    executor.add_node(action_node)
    executor.spin()



if __name__ == '__main__':
    main()