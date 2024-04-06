from ltl_automaton_messages.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLStateArray
from ltl_automaton_messages.srv import ClosestState, TaskPlanning
from ltl_automaton_planner.ltl_tools.buchi import *
from ltl_automaton_planner.ltl_tools.discrete_plan import *
from ltl_automaton_planner.ltl_tools.ltl_planner import *
from ltl_automaton_planner.ltl_tools.ltl2ba import *
from ltl_automaton_planner.ltl_tools.product import *
from ltl_automaton_planner.ltl_tools.promela import *
from ltl_automaton_planner.ltl_tools.ts import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Header, Bool
import os
from rclpy.executors import MultiThreadedExecutor 
from ltl_automaton_planner.nodes.planner_node import show_automaton
from ament_index_python.packages import get_package_prefix
import yaml

import rclpy
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('prova')
    
    
    
    
    
    
    
    '''file_name = "test_ts"
    print(" File can be find at %s" % os.path.join(get_package_prefix('ltl_automaton_std_transition_systems'),
                                                                            'ltl_automaton_std_transition_systems',
                                                                             'config',
                                                                             'generated_ts',
                                                                             file_name+'.yaml'))
    path= os.path.join(get_package_prefix('ltl_automaton_std_transition_systems').replace('/install/', '/src/'),
                                                                            'ltl_automaton_std_transition_systems',
                                                                             'config',
                                                                             file_name+'.yaml')
    
    print(path)
    #file_path = path.replace('/install/', '/src/')
    print(os.path.exists(path))
    #workspace_path = os.getenv('AMENT_PREFIX_PATH')
    #print(workspace_path)
    
    executor = MultiThreadedExecutor() 
    node = rclpy.create_node('prova')
        qos_profile = rclpy.qos.QoSProfile(depth=1, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)   
    point_msg = Point()
    point_msg.x = 1.5
    point_msg.y = -0.87 
    point_msg.z = 0.0
    Quaternion_msg = Quaternion()
    Quaternion_msg.x = 0.0
    Quaternion_msg.y = 0.0
    Quaternion_msg.z = 0.999999998926914
    Quaternion_msg.w = 4.632679487995776e-05  
    pose_msg = Pose()
    pose_msg.position = point_msg
    pose_msg.orientation = Quaternion_msg
    #subscriber1 = node.create_subscription(LTLPlan, 'prefix_plan', lambda msg: print("\n\nprefix: "+str(msg)), qos_profile)
    #subscriber2 = node.create_subscription(LTLPlan, 'suffix_plan', lambda msg: print("\n\nsuffix: "+str(msg)), qos_profile)
    subscriber3 = node.create_subscription(Bool, 'mix_cmd', lambda msg: print("\n\nresult of sub: "+str(msg)), qos_profile)
    subscriber4 = node.create_subscription(String, 'next_move_cmd', lambda msg: print("\n\nnext_move_cmd: "+str(msg)), qos_profile)
        
    publisher1= node.create_publisher(TransitionSystemStateStamped, 'ts_state', qos_profile)
    
    publisher2= node.create_publisher(Bool, 'key_cmd', qos_profile)
    publisher3= node.create_publisher(Bool, 'planner_cmd', qos_profile)
    
    #client = node.create_client(TaskPlanning, 'replanning')
    TSSS_msg = TransitionSystemStateStamped()
    TSS_msg = TransitionSystemState()
    TSS_msg.states = ["r1", "unloaded"]
    TSS_msg.state_dimension_names= ["2d_pose_region", "nexus_load"]
    header_msg = Header()
    header_msg.stamp = node.get_clock().now().to_msg()
    TSSS_msg.header = header_msg
    TSSS_msg.ts_state = TSS_msg
    bool_msg = Bool()
    bool_msg.data = True
    #string_msg.data = "s2"
    print("publish 1")
    publisher1.publish(TSSS_msg)
    print("publish 2")
    publisher2.publish(bool_msg) 
    #print("publish 2")
    #publisher1.publish(TSSS_msg)    
    
    #request = TaskPlanning.Request()
    #request.hard_task = "<>[]r2"

    #client.wait_for_service()
    #future= client.call_async(request)
    #rclpy.spin_until_future_complete(node, future)
    #print(future.result())
    
    #working solution
    #future = client.call_async(request)
    #future.add_done_callback(lambda future: print(future.result()))
    
    #while future.done()==False:
        #print("waiting")

    #try:
        #result = future.result()
    #except Exception as e:
        #node.get_logger().info('Service call failed %r' % (e,))
        
    #print(future)
    #print(result)
    #message=TransitionSystemStateStamped()
    #message.header.stamp = node.get_clock().now().to_msg()
    #ts_state_msg = TransitionSystemState()
    #ts_state_msg.states = ["r2", "unloaded"]
    #ts_state_msg.state_dimension_names= ["2d_pose_region", "turtlebot_load"]
    #message.ts_state = ts_state_msg

    rclpy.spin(node, executor)
    
    
    '''

    
if __name__ == '__main__':
    main()
    
    
