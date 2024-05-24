import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from  rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ltl_automaton_synchronization.transition_systems.ts_definitions import MotionTS, ActionModel, MotActTS
from ltl_automaton_msg_srv.msg import TransitionSystemStateStamped, TransitionSystemState, LTLPlan, LTLState, LTLStateArray
from ltl_automaton_msg_srv.srv import FinishCollab
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point, Quaternion
import yaml
from ltl_automaton_msg_srv.msg import SynchroConfirm

from ltl_automaton_synchronization.waypoint_chaser.mpc_turtlebot import MPC_Turtlebot
from ltl_automaton_synchronization.waypoint_chaser.mpc_rosie import MPC_Rosie
import numpy as np
import math

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
        
        # creating obstacles regions dictionary
        obstacles_dictionary_path = self.declare_parameter('obstacles_dictionary_path', '').value
        
        self.dynamic_obstacles_regions={}
        self.static_obstacles_regions={}
        with open(obstacles_dictionary_path, 'r') as file:
            obstacles_regions = yaml.safe_load(file)
            for (obstacle, region) in obstacles_regions.items():
                # we conside dynamic only the one starting with / since they are the one measured by the mocap
                if obstacle.startswith('/'):
                    self.dynamic_obstacles_regions[obstacle] = region
                else:
                    self.static_obstacles_regions[obstacle] = (region)
            
        # name of the current agent
        self.agent = self.get_namespace()
        
        #initial pose of the agent [x, y, theta]
        self.current_pose = [0, 0, 0] 
        
        # list of agents names
        self.agents = self.declare_parameter('agents', ['']).value
        
        # list of obstacles names to use in conjucntion with mocap in includes agents and
        # other obstacles
        self.dynamic_obstacles = self.declare_parameter('dynamic_obstacles', ['']).value
        
        # agents involved in the collaboration
        self.collaborative_agents = {'master':'', 'assisting_agents':[]}
        # number of assising agents who are ready to start collaborating
        self.agents_ready = 0
        # flag to indicate I can start the assisitve action
        self.start_assising_flag = False
        # flag to understand if we are in a simulation or in areal experiment
        self.is_simulation = self.declare_parameter('is_simulation', True).value
    
    def build_automaton(self):
        
        # motion and actions
        motion_ts = MotionTS(self.motion_dict)
        action_mod = ActionModel(self.action_dict)
     
        # Here we take the product of each element of state_models to define the full TS
        self.robot_model = MotActTS(motion_ts, action_mod)
        self.robot_model.build_full()
        
        # updating action dictionary with new actions after building the full TS
        self.action_dict = self.robot_model.graph['action'].action
        #setting initial state
        self.current_state = list(self.robot_model.graph['initial'])[0]
        

    def setup_pub_sub(self):
        self.qos_profile = rclpy.qos.QoSProfile(depth=1, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publisher for the current state
        self.state_pub = self.create_publisher(TransitionSystemStateStamped, 'ts_state', self.qos_profile) 

        # added callback group to guarantee functioning of the other publishers and subscribers
        action_cb_group = MutuallyExclusiveCallbackGroup()
        # Subscriber for the actions to be executed
        self.action_sub = self.create_subscription(String, 'next_move_cmd', self.action_callback, self.qos_profile, callback_group=action_cb_group)

        # subscribing to confirmation to get the agents involved in the collaboration
        self.confirm_sub = self.create_subscription(SynchroConfirm, '/synchro_confirm', self.confirm_callback, self.qos_profile)

        # Subscriber for the synchro ready message
        ready_cb_group = ReentrantCallbackGroup()
        self.qos_profile_ready = rclpy.qos.QoSProfile(depth=len(self.agents), history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.synchro_ready_sub = self.create_subscription(String, 'synchro_ready', self.synchro_ready_callback, self.qos_profile_ready, callback_group=ready_cb_group)

        # Subscriber for the synchro start message
        self.synchro_start_sub = self.create_subscription(String, 'synchro_start', self.synchro_start_callback, self.qos_profile)
    
        # creating publishers to say that i'm ready to other agents and adding them to a dictionary
        self.synchro_ready_pubs = {}
        for agent in self.agents:
            self.synchro_ready_pubs[agent]= self.create_publisher(String, agent+'/synchro_ready', self.qos_profile)

        # creating publishers to say that i'm ready to other agents and adding them to a dictionary
        self.synchro_start_pubs = {}
        for agent in self.agents:
            self.synchro_start_pubs[agent]= self.create_publisher(String, agent+'/synchro_start', self.qos_profile)

        # Set trap check service client
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.finish_collab_srv = self.create_client(FinishCollab, "finished_collab", callback_group=client_cb_group)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # MOCAP subscribers
        mocap_cb_group = ReentrantCallbackGroup()
        self.obs_sub={}
        for obstacle in self.dynamic_obstacles:
            self.obs_sub[obstacle] = self.create_subscription(PoseStamped, '/qualisys'+obstacle+'/pose', lambda msg: self.obstacles_cb(msg, obstacle), 10, callback_group=mocap_cb_group)
        # subscriber fot the pose of the robot
        self.current_pose = self.create_subscription(PoseStamped, '/qualisys'+self.agent+'/pose', self.update_pose_callback , 10, callback_group=mocap_cb_group)
    
    
    def action_callback(self, msg):
        next_state = ''
        weight = 0
        action_label = msg.data
        
        # finding the next state and the weight of the action
        for state in self.robot_model.successors(self.current_state):
            if self.robot_model[self.current_state][state]['action'] == action_label:
                next_state = state                
                weight = self.robot_model[self.current_state][state]['weight']
        # getting action key
        action_key = self.key_given_label(action_label)
        # check the type of action 
        if self.action_dict[action_key]['type'] == 'local':
            #select the action that needs to be executed
            self.select_action(action_key, weight)
        elif self.action_dict[action_key]['type'] == 'collaborative':
            # collaborative action start master protocol
            self.start_collaboration(self.collaborative_agents['assisting_agents'], weight)
            # updating collaborative agents dictionary
            self.collaborative_agents = {'master':'', 'assisting_agents':[]}                
        else:
            # assistive action start assisting protocol
            self.start_assising(self.collaborative_agents['master'], weight)
            # updating collaborative agents dictionary
            self.collaborative_agents = {'master':'', 'assisting_agents':[]}
        
        # settimg up the message to be published
        next_state_msg = TransitionSystemStateStamped()
        # add state to the message
        next_msg = TransitionSystemState()
        next_msg.states = [next_state[0], next_state[1]]
        next_msg.state_dimension_names = ['MotionTS', 'ActionModel']
        next_state_msg.ts_state = next_msg
        # add header to the message
        header_msg = Header()
        header_msg.stamp = self.get_clock().now().to_msg()
        next_state_msg.header = header_msg        
        # confirming the action has been completed and publishing new state
        self.get_logger().info('ACTION NODE: Publishing State: %s. After Action: %s' %(str(next_state), msg.data))
        self.state_pub.publish(next_state_msg)
        
        # updating the planner saying the collaborative action has ended (needed only for the master)
        # slave agents finish a collaboration at the end of the detour
        if self.action_dict[action_key]['type'] == 'collaborative':            
            # Call service to update planner after collaboration
            self.finish_collab_srv.wait_for_service()
            self.finish_collab_srv.call(FinishCollab.Request())
            

    def start_collaboration(self, assising_agents, weight):
        self.get_logger().warn('ACTION NODE: Collaborative action and waiting for other agents to be ready')
        # wait until all assisting agents are ready 
        while self.agents_ready<len(assising_agents):
            pass
        # sending start message to all assisitng agents involved
        self.get_logger().warn('ACTION NODE: Collaborative action started')
        for agent in assising_agents:
            self.synchro_start_pubs[agent].publish(String(data=self.agent))
        
        # resetting the flag
        self.agents_ready = 0
        
        # executing the action, we just wait for now
        start_time=self.get_clock().now().to_msg().sec
        while(self.get_clock().now().to_msg().sec-start_time<weight):
            pass
    
    
    def start_assising(self, master, weight):
        self.get_logger().warn('ACTION NODE: Assisitve action, waiting for starting message')
        #send ready to master
        self.synchro_ready_pubs[master].publish(String(data=self.agent))

        # wait until a confirmation is given by the master
        while not self.start_assising_flag:
            pass
        # resetting the flag
        self.start_assising_flag = False
        
        # executing the action, we just wait for now
        start_time=self.get_clock().now().to_msg().sec
        while(self.get_clock().now().to_msg().sec-start_time<weight):
            pass
        
    
    
    def confirm_callback(self, msg):
        master, time, confirm = self.unpack_confirm_msg(msg)
        # if agent involved build the dictionary of agents involved
        if self.agent == master or self.agent in confirm.keys():
            self.collaborative_agents['master'] = master
            self.collaborative_agents['assisting_agents'] = [agent for agent in confirm.keys()]
        
        
    def unpack_confirm_msg(self, msg):
        confirm = {}
        for i in range(len(msg.agents)):
            agent = msg.agents[i]
            confirm[agent] = (msg.roi[i], msg.actions[i])
        return msg.master, msg.time, confirm   
    
    
    def synchro_ready_callback(self, msg):
        self.get_logger().info('ACTION NODE: Agent %s is ready to give assistance' %msg.data)
        self.agents_ready += 1

    def synchro_start_callback(self, msg):        
        self.get_logger().info('ACTION NODE: Agent %s is ready to start the collaborative action' %msg.data)
        self.start_assising_flag = True

    def select_action(self, action_key, weight):
        #check if we are not in a simulation
        if not self.is_simulation and action_key.startswith('goto'):
            region = action_key.split('_')[2]
            x, y, radius = self.motion_dict['regions'][region]['pose']
            x_0 =  self.current_pose
            #x_0 = np.array([0, 0, np.pi/3]).reshape(3, 1)
            x_t = np.array([x, y, 0]).reshape(3, 1)    
            # MPC object based on the robot we are using
            if self.get_namespace().startswith('/turtlebot'):
                mpc = MPC_Turtlebot(x_0, x_t)
            else:
                mpc = MPC_Rosie(x_0, x_t)
            # update obstacles
            obstacles = self.list_obstacles()
            #obstacles = [[0, 1, 0]]
            # update constraints
            mpc.update_constraints(obstacles)
            # looping until i'm inside the region but with a smaller radius
            while np.sqrt((mpc.x_0[0]-mpc.x_t[0])** 2+(mpc.x_0[1]-mpc.x_t[1])** 2)>0.7*radius:            
                # mpc iteration
                control = mpc.get_next_control()
                # publish control
                self.publish_vel_control(control)
                # update obstacles
                obstacles = self.list_obstacles()
                # update constraints
                mpc.update_constraints(obstacles)
                #update pose
                mpc.x_0 = self.current_pose
            # stop the robot at the end
            if self.get_namespace().startswith('/turtlebot'):
                self.publish_vel_control([0.0, 0.0])
            else:
                self.publish_vel_control([0.0, 0.0, 0.0])   
        else:
            start_time=self.get_clock().now().to_msg().sec
            while(self.get_clock().now().to_msg().sec-start_time<weight):
                pass

                
    def list_obstacles(self):
        obs = list(self.static_obstacles_regions.values())
        for obstacle in self.dynamic_obstacles:            
            obs.append(self.dynamic_obstacles_regions[obstacle])
        return obs            
                
                
    def publish_vel_control(self, u):
        # for turtlebots
        if self.get_namespace().startswith('/turtlebot'):
            # linear velocity
            linear = Vector3()
            #linear.x = np.float64(u[0])
            linear.x = u[0]
            #angular velocity
            angular = Vector3()
            #angular.z = np.float64(u[1])
            angular.z = u[1]
            # Twist message
            msg = Twist()
            msg.linear = linear
            msg.angular = angular
        # for rosies
        else:
            # linear velocity on x
            linear = Vector3()
            linear.x = u[0]
            # linear velocity on y
            linear.y = u[1]
            #angular velocity
            angular = Vector3()
            angular.z = u[2]
        # Twist message
        msg = Twist()
        msg.linear = linear
        msg.angular = angular
        # publish the command
        self.cmd_pub.publish(msg)
    
    def key_given_label(self, label):
        for key, value in self.action_dict.items():
            if value['label'] == label:
                return key
        return None

    def obstacles_cb(self, msg, obstacle):
        # update the region definition for the obstacle if valid        
        if(not math.isnan(msg.pose.position.x)):
            self.dynamic_obstacles_regions[obstacle] = [msg.pose.position.x, msg.pose.position.y, self.dynamic_obstacles_regions[obstacle][2]]
    
    
    def update_pose_callback(self, msg):
        if(not math.isnan(msg.pose.position.x)):
            self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y, np.unwrap([2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)])[0]]).reshape(3, 1)  








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