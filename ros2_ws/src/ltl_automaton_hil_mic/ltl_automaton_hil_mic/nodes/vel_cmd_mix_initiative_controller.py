#!/usr/bin/env python

#TODOD: this node has not been tested but everything seems to be in order and all the adaptations
# to work with ROS2 have been made.
 
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from  rclpy.callback_groups import ReentrantCallbackGroup
import math
import numpy
from copy import deepcopy
from geometry_msgs.msg import Twist
import sys

#Import LTL automaton message definitions
from ltl_automaton_messages.msg import TransitionSystemStateStamped, TransitionSystemState
from ltl_automaton_messages.srv import ClosestState, TrapCheck

#==============================================
#         Mix Initative Controller object
#            for Velocity Commands
#                     ---
# Takes as input velocity commands (Twist msg)
# and a distance to trap metric to mix planner
#             and human inputs
#==============================================
class VelCmdMixer(Node):
    def __init__(self):
        super().__init__('vel_mic_hil_controller')
        # Get parameters
        self.load_params()

        # Setup subscribers and publishers
        self.set_pub_sub()

        self.get_logger().info("Human-in-the-loop velocity commands mix-initiative controller initialized")

    #--------------------------------------------------
    # Load controller parameters
    #--------------------------------------------------
    def load_params(self):
        self.curr_ts_state = None

        # Get epsilon gain from ROS parameters
        self.epsilon = self.declare_parameter("epsilon", 1.5).value

        # Get safety distance from ROS parameters
        self.ds = self.declare_parameter("ds", 1.2).value

        # Get deadband from ROS parameters
        self.deadband = self.declare_parameter("deadband", 0.2).value

        # Get human input timeout (in seconds) from ROS parameters
        self.timeout = self.declare_parameter("timeout", 0.2).value*1e9
        # Init last received human input time
        self.last_received_human_input = 0

        # Get velocity component saturations
        self.max_linear_x_vel = self.declare_parameter("max_linear_x_vel", 0.5).value
        self.max_linear_y_vel = self.declare_parameter("max_linear_y_vel", 0.5).value
        self.max_linear_z_vel = self.declare_parameter("max_linear_z_vel", 0.5).value
        self.max_angular_x_vel = self.declare_parameter("max_angular_x_vel", 2.0).value
        self.max_angular_y_vel = self.declare_parameter("max_angular_y_vel", 2.0).value
        self.max_angular_z_vel = self.declare_parameter("max_angular_z_vel", 2.0).value

        # Node frequency
        self.frequency = self.declare_parameter("node_frequency", 50).value

        # Get monitored TS state model
        self.state_dimension_name = self.declare_parameter("state_dimension_name", "2d_pose_region").value

        supported_state_types = ["2d_pose_region",
                                 "3d_pose_region",
                                 "2d_point_region",
                                 "3d_point_region"]
        
        if self.state_dimension_name not in supported_state_types:
            raise ValueError("TS state dimension %s is not supported by velocity command mix initiave controller")

    #---------------------------------------------------
    # Setup subscribers, publishers and service clients
    #---------------------------------------------------
    def set_pub_sub(self):
        
        # define QoS profile for publishers
        # depth and history are used to set a quiesize of 1 while durability is to store messages for late subscribers
        self.qos_profile = rclpy.qos.QoSProfile(depth=50, history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST)
        self.callback_group = ReentrantCallbackGroup()
        
        # Set closest region service client
        self.closest_reg_srv = self.create_client(ClosestState, "closest_region", callback_group=self.callback_group)

        # Set trap check service client
        self.trap_cheq_srv = self.create_client(TrapCheck, "check_for_trap", callback_group=self.callback_group)

        # Set mix initiave controller output
        self.mix_vel_cmd_pub = self.create_publisher(Twist, "cmd_vel", self.qos_profile)

        # Set agent TS state subscriber
        self.ts_state_sub = self.create_subscription(TransitionSystemStateStamped, "ts_state", self.ts_state_callback, self.qos_profile)

        # Set human input planner
        self.key_vel_sub = self.create_subscription(Twist, "key_vel", self.teleop_cmd_callback, self.qos_profile)

        # Set planner input subscriber
        self.nav_vel_sub = self.create_subscription(Twist, "nav_vel", self.planner_cmd_callback, self.qos_profile, callback_group=self.callback_group)

    #-------------------------
    # Agent TS state callback
    #-------------------------
    def ts_state_callback(self, msg=TransitionSystemStateStamped()):
        # If not using same state model type, print warning and ignore message
        if not self.state_dimension_name in msg.ts_state.state_dimension_names:
            self.get_logger().warning("Received TS state does not include state model type used by velocity command HIL MIC (%s), TS state is of type %s"
                          % (self.state_dimension_name, msg.ts_state.state_dimension_names))
        # If lenght of states is different from length of state dimension names, message is malformed
        # print warning and ignore message
        if not (len(msg.ts_state.state_dimension_names) == len(msg.ts_state.states)):
            self.get_logger().warning("Received TS state but number of states: %i doesn't correpond to number of state dimensions: %i"
                          % (len(msg.ts_state.states),len(msg.ts_state.state_dimension_names)))
        # Else message is valid, save it
        else:
            self.curr_ts_state = msg.ts_state

    #--------------------------------
    # Planner velocity command input
    #--------------------------------
    def planner_cmd_callback(self, msg=Twist()):
        self.planner_input_vel = msg

        # If human input received recently and TS state is known
        if self.last_received_human_input:
            if ( self.get_clock().now().to_msg.nanosec - self.last_received_human_input < self.timeout) and (self.curr_ts_state):
                # Run controller mix and publish
                self.mix_vel_cmd_pub.publish(self.control_mixer(self.teleop_input_vel, self.planner_input_vel))
                return
        
        # Else use planner input directly
        self.mix_vel_cmd_pub.publish(self.planner_input_vel)

    #--------------------------------
    # Human velocity command input
    #--------------------------------
    def teleop_cmd_callback(self, msg=Twist()):
        self.teleop_input_vel = msg
        self.last_received_human_input = self.get_clock().now().to_msg.nanosec

    #------------------------------------------------
    # Check if risk of trap and get distance to trap
    #------------------------------------------------
    def check_for_trap(self):
        # Get closest region
        closest_reg_req = ClosestState.Request()
        
        # Call service to get the closest region
        self.closest_reg_srv.wait_for_service()
        closest_reg = self.closest_reg_srv.call(closest_reg_req)


        # If service returns a closest state
        if closest_reg.closest_state:
            # Create check for trap request from TS state
            ts_state_to_check = deepcopy(self.curr_ts_state)
            for i in range(len(self.curr_ts_state.state_dimension_names)):
                # Replace current region by region to check
                if self.curr_ts_state.state_dimension_names[i] == self.state_dimension_name:
                    ts_state_to_check.states[i] = closest_reg.closest_state

            # Populate request and call service to check if closest region would trigger a trap state
            check_for_trap_req = TrapCheck.Request()
            check_for_trap_req.ts_state = ts_state_to_check
            
            # Call service to check if a state is a trap
            self.trap_cheq_srv.wait_for_service()
            check_for_trap_res = self.trap_cheq_srv.call(check_for_trap_req)


            self.get_logger().debug("LTL velocity command MIC: Closest region is %s and distance is %f" % (closest_reg.closest_state, closest_reg.metric))

            # if IT'S A TRAP! (insert Amiral Ackbar meme)
            if check_for_trap_res.is_connected and check_for_trap_res.is_trap:
                self.get_logger().debug("LTL velocity command MIC: Agent is in state %s and state %s is a trap" % (self.curr_ts_state.states, check_for_trap_req.ts_state.states))
                # Return distance to closest region
                return closest_reg.metric

        # If not a trap or cannot check for trap (no closest region or region unconnected)
        return None
    
    #------------------------------------------------------------
    # Mix human input and planner input according to agent state
    #------------------------------------------------------------
    def control_mixer(self, teleop_vel_cmd, planner_vel_cmd):
        #print 'telecontrol signal is' + str(self.tele_control
        tele_magnitude = max(math.sqrt(teleop_vel_cmd.linear.x**2
                                     + teleop_vel_cmd.linear.y**2
                                     + teleop_vel_cmd.linear.z**2),
                             math.sqrt(teleop_vel_cmd.angular.x**2
                                     + teleop_vel_cmd.angular.y**2
                                     + teleop_vel_cmd.angular.z**2))

        if tele_magnitude >= self.deadband:
            self.get_logger().debug("LTL velocity command MIC: Human inputs detected")

            # Check for trap
            dist_to_trap = self.check_for_trap()

            # If dist to trap returns a value, compute mix
            if not (dist_to_trap == None):
                # print 'Distance to trap states in product: %.2f' % (dist_to_trap)
                teleop_vel_cmd = self.bound_vel_cmd(teleop_vel_cmd)
                mix_control = self.smooth_mix(teleop_vel_cmd, planner_vel_cmd, dist_to_trap, self.ds, self.epsilon)
                # print '---mix_control---:'
                # print mix_control
                # print "---------"
                # print '---navi_control---'
                # print planner_vel_cmd
                # print "---------"
                # print '---tele_control---'
                # print teleop_vel_cmd
                # print "---------"
            
            # If distance to trap returns false, no trap is close, use human input
            else:
                #if no obstacle is close, use human command
                self.get_logger().debug("LTL velocity command MIC: No trap states are close")
                teleop_vel_cmd = self.bound_vel_cmd(teleop_vel_cmd)
                mix_control = teleop_vel_cmd
        else:
            self.get_logger().debug("LTL velocity command MIC: Human inputs below deadband. Autonomous controller used.")
            mix_control = planner_vel_cmd

        return mix_control

    #------------------------------------------------------------------------
    # Return the mixed velocity command given the distance to trap and gains
    #------------------------------------------------------------------------
    def smooth_mix(self, tele_control, navi_control, dist_to_trap, ds, epsilon):
        # Compute gain using epsilon, dist to trap and ds
        gain = self.rho(dist_to_trap-ds)/(self.rho(dist_to_trap-ds)+self.rho(epsilon +self.ds-dist_to_trap))
        self.get_logger().debug("LTL velocity command MIC: human-in-the-loop gain is " + str(gain))

        # Mix velocity commands by using previously calculated gain
        mix_vel_cmd = Twist()
        mix_vel_cmd.linear.x = (1 - gain) * navi_control.linear.x + gain * tele_control.linear.x
        mix_vel_cmd.linear.y = (1 - gain) * navi_control.linear.y + gain * tele_control.linear.y
        mix_vel_cmd.linear.z = (1 - gain) * navi_control.linear.z + gain * tele_control.linear.z
        mix_vel_cmd.angular.x = (1 - gain) * navi_control.angular.x + gain * tele_control.angular.x
        mix_vel_cmd.angular.y = (1 - gain) * navi_control.angular.y + gain * tele_control.angular.y
        mix_vel_cmd.angular.z = (1 - gain) * navi_control.angular.z + gain * tele_control.angular.z

        return mix_vel_cmd

    #-----------------------------------------------------------
    # Bound the minimum and maximum value of a velocity command
    #-----------------------------------------------------------
    def bound_vel_cmd(self, vel_twist_msg):
        vel_twist_msg.linear.x = self.bound(vel_twist_msg.linear.x, self.max_linear_x_vel)
        vel_twist_msg.linear.y = self.bound(vel_twist_msg.linear.y, self.max_linear_y_vel)
        vel_twist_msg.linear.z = self.bound(vel_twist_msg.linear.z, self.max_linear_z_vel)
        vel_twist_msg.angular.x = self.bound(vel_twist_msg.angular.x, self.max_angular_x_vel)
        vel_twist_msg.angular.y = self.bound(vel_twist_msg.angular.y, self.max_angular_y_vel)
        vel_twist_msg.angular.z = self.bound(vel_twist_msg.angular.z, self.max_angular_z_vel)

        return vel_twist_msg

    #---------------
    # Bound a value
    #---------------
    def bound(self, command, max_value):
        if command > max_value:
            command = max_value
        elif command < -max_value:
            command = -max_value

        return command

    #-------------------------------------
    # Rho function needed for mix formula
    #-------------------------------------
    def rho(self, s):
        if (s > 0):
            return numpy.exp(-1.0/s)
        else:
            return 0


#============================
#            Main            
#============================
def main():
    rclpy.init()        
    try: 
        vel_mic_node = VelCmdMixer()          
        executor = MultiThreadedExecutor() 
        executor.add_node(vel_mic_node) 
        executor.spin() 
    except ValueError as e:
        rclpy.logging.get_logger("vel_cmd_hil_mic").error("Velocity Command HIL MIC: %s" %(e))
        sys.exit(0)


if __name__ == '__main__':
    main()