#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
import csv
import math
import numpy as np
from numpy import sin, cos
import time
import tf
import threading
import os
from os.path import expanduser

from rm3_planning_and_control.srv import Status, StatusResponse, SetActionMode
from std_srvs.srv import SetBool, SetBoolResponse

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix

import actionlib

from rm3_planning_and_control.msg import MotionAction, MotionGoal, MotionResult

from enum import Enum

def pose2d_from_odom(odom):

    pos_des_x = odom.pose.pose.position.x
    pos_des_y = odom.pose.pose.position.y

    (roll, pitch, yaw_des) = euler_from_quaternion([odom.pose.pose.orientation.x,
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])

    # return MotionGoal(Point(pos_des_x, pos_des_y,yaw_des))
    return Pose2D(pos_des_x, pos_des_y,yaw_des)

def odometry_from_pose(goal_pose_stamped_msg):
    odometry_goal = Odometry()

    odometry_goal.pose.pose.position.x = goal_pose_stamped_msg.pose.position.x
    odometry_goal.pose.pose.position.y = goal_pose_stamped_msg.pose.position.y

    odometry_goal.pose.pose.orientation = goal_pose_stamped_msg.pose.orientation

    return odometry_goal

class Mode(Enum):
    STOPPED = 0
    FORMULA_1 = 1
    FORMULA_2 = 2
    FORMULA_3 = 3
    MOVING_FORWARD=4
    MOVING_BACKWARD=5

class SimpleMRMotionNode:
    """SimpleMRMotionNode class"""

    def __init__(self,odometry_topic, controller_prefix, goal_topic, action_name, robot_name, enable_voice_command):

        self.mutex = threading.Lock()
        self.odometry_topic = odometry_topic
        self.controller_prefix = controller_prefix
        self.robot_name = robot_name
        self.mode = Mode.FORMULA_1

        with self.mutex:
            self.current_command_finished = True

            # Action clients
            # self.a_client= actionlib.SimpleActionClient(controller_prefix + action_name, MotionAction)
            # self.a_client.wait_for_server()

            self.timeout = 100

            # Create topics
            rospy.Subscriber("/ltl/"+ robot_name +"/goal_pose", PoseStamped, self.goal_pose_callback)
            self.goal_pub = rospy.Publisher(goal_topic, Odometry, queue_size=1, latch=True)

            if enable_voice_command:
                rospy.Subscriber("/Command", String, self.command_callback)
                rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback)

            # Change controller mode: (0) topics, (1) action server, (2) action-topic
            self.controller_mode = 0
            self.set_controller_mode(controller_prefix, self.controller_mode)

            self.odom_goal = None

    ## NAVIGATION SERVICES ##

    def set_controller_mode(self, prefix, mode:int):
        rospy.loginfo("Waiting for service: %s", prefix+'/change_action_mode')
        rospy.wait_for_service(prefix + '/change_action_mode')
        try:
            controller_mode_srv = rospy.ServiceProxy(prefix + '/change_action_mode', SetActionMode)
            rospy.loginfo("Changing controller for %s", self.robot_name)
            controller_mode_result = controller_mode_srv(mode)
            print(f"Changed controller for {self.robot_name}: ", controller_mode_result.message)
        except rospy.ServiceException as e:
            print("Change controller mode call failed: %s"%e)

    def set_planner_status(self, prefix, new_status):
        #planner status call new_status=[idle=0, nav=1, dock=2]
        rospy.wait_for_service(prefix+'/change_status')
        try:
            plan_status_client = rospy.ServiceProxy(prefix+'/change_status', Status)
            resp = plan_status_client(new_status)
        except rospy.ServiceException as e:
            print("Change status service call failed: %s"%e)

    def set_activate_controller(self, prefix, new_status):
        #activate controller call new_status=[off=False, on=True]
        rospy.wait_for_service(prefix+'/activate_controller')
        try:
            act_cntr_client = rospy.ServiceProxy(prefix+'/activate_controller', SetBool)
            resp = act_cntr_client(new_status)
        except rospy.ServiceException as e:
            print("Activate Controller service call failed: %s"%e)

    ## CALLBACKS FOR TOPICS ##

    def goal_pose_callback(self, goal_pose_stamped_msg):
        with self.mutex:

            if self.mode != Mode.FORMULA_1:
                return

            #Save when last pose was received
            self.goal_received_at = rospy.Time.now()
            self.move_message = f"Moving to {goal_pose_stamped_msg.pose.position.x:.2f}, {goal_pose_stamped_msg.pose.position.y:.2f}"
            # rospy.loginfo(self.move_message)

            self.odom_goal = odometry_from_pose(goal_pose_stamped_msg)

            self.send_odom_goal(self.odom_goal)

    def odometry_callback(self, odom_msg):
        with self.mutex:
            if self.mode == Mode.MOVING_FORWARD or self.mode == Mode.MOVING_BACKWARD:
                goal = odom_msg
                (_, _, yaw) = euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                                            odom_msg.pose.pose.orientation.y,
                                                            odom_msg.pose.pose.orientation.z,
                                                            odom_msg.pose.pose.orientation.w])

                dist = 2
                if self.mode == Mode.MOVING_BACKWARD:
                    dist = -dist
                goal.pose.pose.position.x += dist * cos(yaw)
                goal.pose.pose.position.y += dist * sin(yaw)

                self.send_odom_goal(goal)

    def command_callback(self, command_msg):
        with self.mutex:
            command = command_msg.data
            if command == "formula_one":
                self.mode = Mode.FORMULA_1
                rospy.loginfo("Switching to FORMULA_1")
            elif command == "formula_two":
                self.mode = Mode.FORMULA_2
                rospy.loginfo("Switching to FORMULA_2")
            elif command == "formula_three":
                self.mode = Mode.FORMULA_3
                rospy.loginfo("Switching to FORMULA_3")
            elif command == "forward_1":
                self.mode = Mode.MOVING_FORWARD
                rospy.loginfo("Switching to MOVING_FORWARD")
            elif command == "backward_1":
                self.mode = Mode.MOVING_BACKWARD
                rospy.loginfo("Switching to MOVING_BACKWARD")
            elif command == "start":
                self.mode = Mode.FORMULA_1
                rospy.loginfo("Switching to FORMULA_1 (Start)")
            elif command == "stop":
                self.mode = Mode.STOPPED
                rospy.loginfo("Switching to STOPPED")
                self.set_activate_controller(self.controller_prefix, False)
            else:
                rospy.loginfo(f"Unknown command: {command_msg}")

    def send_odom_goal(self, goal):
        modality = 1 #1 is for navigation, 2 for docking
        # Sends the goal to the goal_topic, activate controller with proper mode.
        self.set_controller_mode(self.controller_prefix, self.controller_mode)
        point_goal = pose2d_from_odom(goal)
        rospy.loginfo(f"sending to {self.robot_name}: {point_goal.x:.2f}, {point_goal.y:.2f}, yaw = {math.degrees(point_goal.theta):.2f}")
        self.goal_pub.publish(goal)
        self.set_planner_status(self.controller_prefix, modality)
        self.set_activate_controller(self.controller_prefix,True)

    ## MAIN LOOP: MOTION LOGIC ##
    def run_motion_loop(self):
        with self.mutex:
            if self.mode not in [Mode.FORMULA_1, Mode.FORMULA_2, Mode.FORMULA_3]:
                return
            # if not self.current_command_finished:
            if self.odom_goal is not None:
                self.send_odom_goal(self.odom_goal)

            # self.current_command_finished = True

    def run(self):
        rate = rospy.Rate(0.25)
        # self.motion_init()
        while not rospy.is_shutdown():
            self.run_motion_loop()
            rate.sleep()

def run_motion():

    #ROS init
    rospy.init_node('MRmotion', anonymous=True)

    odometry_topic=rospy.get_param('~odometry_topic')
    controller_prefix=rospy.get_param('~controller_prefix')
    action_name=rospy.get_param('~action_name')
    robot_name=rospy.get_param('~robot_name')
    goal_topic=rospy.get_param('~goal_topic')
    enable_voice_command=0#rospy.get_param('~enable_voice_command')

    mrmotion_node = SimpleMRMotionNode(odometry_topic, controller_prefix, goal_topic, action_name, robot_name, enable_voice_command=enable_voice_command)
    mrmotion_node.run()

if __name__ == '__main__':
    try:
        run_motion()
    except rospy.ROSInterruptException:
        pass
