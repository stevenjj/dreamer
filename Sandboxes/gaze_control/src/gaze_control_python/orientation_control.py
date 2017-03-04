#!/usr/bin/env python
import rospy
import time
import math
import modern_robotics as mr
import numpy as np
import util_quat as quat
import head_kinematics as hk
import dreamer_joint_publisher
from traj_manager import *

# State Lists
IDLE = 0
GO_TO_POINT = 1
FOLLOWING_TRAJECTORY = 2
FINISHED_TRAJECTORY = 3
INTERRUPTED = 4

# High Level Task Lists
NO_TASK = 100
GAZE_AVERSION = 101
TASK_HIERARCHY = 102
CIRCULAR_HEAD_TRAJ = 103
TRACK_PERSON = 104
TRACK_MARKER = 105

# High Level Behavior List --- seems like this is another high level task list
DO_NOTHING = 200
ALWAYS_GAZE_AVERT = 201
ALWAYS_GAZE_FOLLOW = 202


class Dreamer_Head():
    command_once = False
    def __init__(self):
        self.behaviors = []
        self.tasks = [NO_TASK, GAZE_AVERSION, TASK_HIERARCHY, CIRCULAR_HEAD_TRAJ, TRACK_PERSON, TRACK_MARKER]
        self.states = [IDLE, GO_TO_POINT, FOLLOWING_TRAJECTORY, FINISHED_TRAJECTORY, INTERRUPTED]

        self.current_state = IDLE
        self.current_task = NO_TASK

        self.kinematics = hk.Head_Kinematics() 
        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()

        self.traj_manager = Trajectory_Manager()

        # ROS Loop details
        # Send control messages at 500hz. 10 messages to send per ROS loop so this node operates at 50Hz
        self.node_rate = 1/(500)
        self.rate = rospy.Rate(500) 
        self.ROS_start_time = rospy.Time.now().to_sec()
        self.ROS_current_time = rospy.Time.now().to_sec()

        self.task_list = [0, 1]
        self.current_task = 0

        # READ Current Joint Positions
        # Otherwise send HOME command

    def callback_change_state(self, string):
        #specifying subscriber callback via callback class_instance.method_name
        return 

    def update_head_joints(self, head_joint_list):
        self.kinematics.Jlist = head_joint_list

        def joint_cmd_bound(val, jmax, jmin):
            if val >= jmax:
                return jmax
            elif (val <= jmin):
                return jmin
            else:
                return val

        for i in range(0, len(head_joint_list)):
            command = head_joint_list[i]
            joint_name = self.kinematics.Jindex_to_names[i]
            joint_max_val = self.joint_publisher.free_joints[joint_name]['max']
            joint_min_val = self.joint_publisher.free_joints[joint_name]['min']
            self.joint_publisher.free_joints[joint_name]['position'] = joint_cmd_bound(command, joint_max_val, joint_min_val)

        #print self.kinematics.Jlist


    def task_logic(self):
        self.ROS_current_time = rospy.Time.now().to_sec()
        relative_time =  self.ROS_current_time - self.ROS_start_time
        print "ROS time (sec): ", relative_time            


        # if self.current_task == ()
        # done specifying task

        if ((relative_time > 0.1) and (self.command_once == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time
            Q_cur = self.kinematics.Jlist

            xyz_gaze_loc = np.array([0.5, 0.0, self.kinematics.l1+0.4])            
#            xyz_gaze_loc = np.array([0.5, 0.2, self.kinematics.l1-0.2])                        
#            xyz_gaze_loc = np.array([0.3, 0.3, self.kinematics.l1+0.1])            
#            xyz_gaze_loc = np.array([0.5, 0.5, self.kinematics.l1+0.4])
#            xyz_gaze_loc = np.array([0.3, 0.0, self.kinematics.l1+0.5])
#            xyz_gaze_loc = np.array([0.3, 0.3, self.kinematics.l1+0.0])            

            movement_duration = 4
            self.traj_manager.specify_goal(start_time, Q_cur, xyz_gaze_loc, movement_duration)
            self.command_once = True

        return 

    def behavior_logic(self):
        return 

    def state_logic(self):
        if (self.current_state == IDLE):
            print "STATE = Idle"
        elif (self.current_state == GO_TO_POINT):
            print "STATE = GO_TO_POINT"
            #Q_des, command_result = self.traj_manager.go_to_point()
            Q_des, command_result = self.traj_manager.go_to_point2()
            #Q_des, command_result = self.traj_manager.head_look_at_point()

            #if (command_result == True):
               #self.current_state = IDLE
               #self.command_once = False #+= 1
            self.update_head_joints(Q_des)
        else:
            print "ERROR Not a valid state" 

    def loop(self):
        while not rospy.is_shutdown():
            self.task_logic()
            #self.behavior_logic()
            self.state_logic()          
            # message rate = 500 Hz, num of messages to send = 10
            # node rate = 500/10.0 = 50 hz
            for i in range(0, 10): # 
                # send message
                self.rate.sleep()   #rospy.sleep(1/500.0);    
                # Will sleep for a total of 0.02 seconds --> 50Hz
            self.joint_publisher.publish_joints()


if __name__ == '__main__':
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()