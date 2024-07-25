import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from  rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
from ltl_automaton_msg_srv.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_msg_srv.srv import FinishCollab
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
import yaml
from ltl_automaton_msg_srv.msg import SynchroConfirm

from ltl_automaton_synchronization.waypoint_chaser.mpc_turtlebot import MPC_Turtlebot
from ltl_automaton_synchronization.waypoint_chaser.mpc_rosie import MPC_Rosie
import numpy as np
import math

from rclpy.action import ActionClient
from rosie_pick_and_place_interfaces.action import Pick  # Adjust the import based on your package name
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger




class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')
        self.init_params()
        self.setup_pub_sub()


    def init_params(self):        
        # flag used to check if the pick action has been completed
        self.done_pick = False
        self.done_place = False
        self.agent = '/rosie1'

        

    def setup_pub_sub(self):
        manipulation_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.start_manipulation_sub = self.create_subscription(String , '/rosie1/start_manipulation', self.start_manipulation_cb, 10, callback_group=manipulation_cb_group)
        self.pick_action_client = ActionClient(self, Pick, "/rosie1/pick_action")
        self.place_service = self.create_client(Trigger, "/rosie1/davide_service")
        self.finished_pick_pub = self.create_publisher(String, '/rosie1/finished_pick', 10)
        self.finished_place_pub = self.create_publisher(String, '/rosie1/finished_place', 10)

   
    def start_manipulation_cb(self, msg):
        self.get_logger().warn('ACTION NODE: Starting Picking')
        #call the service to remove the object
        self._send_goal()
        while not self.done_pick:
            pass
        self.done_pick = False 
        # acknowledge the master that the pick has been completed 
        self.get_logger().warn('ACTION NODE: Finished Picking')       
        self.finished_pick_pub.publish(String(data=self.agent))
        self.place_service.wait_for_service()
        self.get_logger().warn('ACTION NODE: Starting Placing')
        self.req = Trigger.Request()
        self.future = self.place_service.call_async(self.req)
        self.future.add_done_callback(self.place_callback)
        while not self.done_place:
            pass
        self.done_place = False
        self.get_logger().warn('ACTION NODE: Finished Placing') 
        self.finished_place_pub.publish(String(data=self.agent))   
   
#==============================
#          Remove Object
#==============================

    def _send_goal(self):
        goal_msg = Pick.Goal()
        # goal_msg.order = 10  # Example goal value

        ## Populate goal message
        goal_msg.ids = ['4']
        # --
        pose1 = Pose()
        pose1.position.x = 0.0
        pose1.position.y = 0.0
        pose1.position.z = 0.0
        pose1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal_msg.aruco_poses = [pose1]
        # --
        
        # define the pregrasp and grasp poses
        quat = Rotation.from_euler("xyz", [180., 0., 90.], degrees=True).as_quat()
        # Create a Quaternion message
        quaternion_msg = Quaternion()
        quaternion_msg.x = quat[0]
        quaternion_msg.y = quat[1]
        quaternion_msg.z = quat[2]
        quaternion_msg.w = quat[3]
        goal_msg.pregrasp_pose = Pose(
            position=Point(x=0., y=0.02, z=0.12),
            orientation=quaternion_msg
        )
        
        goal_msg.grasp_pose = Pose(
            position=Point(x=0., y=0.02, z=0.09),
            orientation=quaternion_msg
        )

        # --
        goal_msg.pregrasp_position_tolerance = Point(x=0.01, y=0.01, z=0.01)
        goal_msg.pregrasp_orientation_tolerance = Point(x=5., y=5., z=5.)
        goal_msg.grasp_position_tolerance = Point(x=0.02, y=0.02, z=0.02)
        goal_msg.grasp_orientation_tolerance = Point(x=5., y=5., z=5.)
        ##
        self.pick_action_client.wait_for_server()
        self._send_goal_future = self.pick_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Success: {result.success}')
        self.done_pick = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.filler}')

    def place_callback(self, future):
        result = future.result()
        if result is not None:
            self.get_logger().info('Result of place: {0}'.format(result.message))
        else:
            self.get_logger().info('Exception while calling service: {0}'.format(future.exception()))
        # Set the flag to True to indicate that the place action has been completed
        self.done_place = True







#==============================
#             Main
#==============================
def main():
    rclpy.init()   
    manipulation_node = ManipulationNode() 
    executor = MultiThreadedExecutor() 
    executor.add_node(manipulation_node)
    executor.spin()



if __name__ == '__main__':
    main()
    
    
