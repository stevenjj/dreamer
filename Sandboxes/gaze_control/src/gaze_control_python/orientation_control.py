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
GO_TO_SOME_POINT = 106
EYE_FOCUS = 107

GO_TO_POINT_A = 108
GO_TO_POINT_B = 109

# High Level Behavior List --- seems like this is another high level task list
DO_NOTHING = 200
ALWAYS_GAZE_AVERT = 201
ALWAYS_GAZE_FOLLOW = 202


class Dreamer_Head():
    command_once = False
    def __init__(self):
        self.behaviors = []
        self.tasks = [GO_TO_SOME_POINT, EYE_FOCUS, NO_TASK, GAZE_AVERSION, TASK_HIERARCHY, CIRCULAR_HEAD_TRAJ, TRACK_PERSON, TRACK_MARKER]
        self.states = [IDLE, GO_TO_POINT, FOLLOWING_TRAJECTORY, FINISHED_TRAJECTORY, INTERRUPTED]

        self.current_state = IDLE
        self.current_task = NO_TASK

        self.kinematics = hk.Head_Kinematics() 
        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()

        self.traj_manager = Trajectory_Manager(self.kinematics)

        # ROS Loop details
        # Send control messages at 500hz. 10 messages to send per ROS loop so this node operates at 50Hz
        self.node_rate = 1/(500)
        self.rate = rospy.Rate(500) 
        self.ROS_start_time = rospy.Time.now().to_sec()
        self.ROS_current_time = rospy.Time.now().to_sec()

        self.task_list = [GO_TO_POINT_A, GO_TO_SOME_POINT, GO_TO_POINT_A, NO_TASK]
        self.current_task_index = 0
        self.task_commanded = False
        self.current_task = self.task_list[self.current_task_index]

        self.current_eye_focal_point = np.array([1,0,0])

        # READ Current Joint Positions
        # Otherwise send HOME command

    def callback_change_state(self, string):
        #specifying subscriber callback via callback class_instance.method_name
        return 

    def update_head_joints(self, head_joint_list):
        self.kinematics.Jlist = head_joint_list

        def joint_cmd_bound(val, joint_name, jmax, jmin):
            if val >= jmax:
                print '    MAX Software Joint HIT! for joint', joint_name
                return jmax
            elif (val <= jmin):
                print '    MIN Software Joint HIT! for joint', joint_name                
                return jmin
            else:
                return val

        for i in range(0, len(head_joint_list)):
            command = head_joint_list[i]
            joint_name = self.kinematics.Jindex_to_names[i]
            joint_max_val = self.joint_publisher.free_joints[joint_name]['max']
            joint_min_val = self.joint_publisher.free_joints[joint_name]['min']
            self.joint_publisher.free_joints[joint_name]['position'] = joint_cmd_bound(command, joint_name, joint_max_val, joint_min_val)

        #print self.kinematics.Jlist


    def next_task(self):
        self.current_task_index += 1
        self.current_task = self.task_list[self.current_task_index]

    def task_logic(self):
        self.ROS_current_time = rospy.Time.now().to_sec()
        relative_time =  self.ROS_current_time - self.ROS_start_time
        print 'ROS time (sec): ', relative_time            


        # TASK GO TO POINT A
        if ((self.current_task == GO_TO_POINT_A) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time
            Q_cur = self.kinematics.Jlist                  
            xyz_gaze_loc = np.array([1.0, 0.3, self.kinematics.l1-0.2])
            movement_duration = 3
            self.traj_manager.specify_gaze_point(start_time, Q_cur, xyz_gaze_loc, movement_duration)  
            self.task_commanded = True
          
        # TASK GO TO SOME POINT
        elif ((self.current_task == GO_TO_SOME_POINT) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time
            Q_cur = self.kinematics.Jlist            
            xyz_gaze_loc = np.array([1.0, -0.4, self.kinematics.l1])
            movement_duration = 3
            self.traj_manager.specify_gaze_point(start_time, Q_cur, xyz_gaze_loc, movement_duration)  
            self.task_commanded = True

        elif ((self.current_task == EYE_FOCUS) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time
            Q_cur = self.kinematics.Jlist            
            movement_duration = 1
            # SPECIFY TRAJ_MANAGER COMMAND self.traj_manager.specify_gaze_point(start_time, Q_cur, xyz_gaze_loc, movement_duration)  
            self.task_commanded = True

    def behavior_logic(self):
        # if behavior == something and self.behavior_set == False
        #   set the task_list = []
        #   initialize task_index = 0
        #   behavior_set = True


        return 

    def process_task_result(self, Q_des, command_result):
        self.update_head_joints(Q_des)
        if (command_result == True):
            self.current_state = IDLE
            self.task_commanded = False
            self.next_task()

    def state_logic(self):
        if (self.current_state == IDLE):
            print '  STATE:', 'Idle'
            if self.current_task == NO_TASK:
                print '  Current_Task:', 'NO Task'
                print '  Current Task Index:', self.current_task_index

        elif (self.current_state == GO_TO_POINT):
            print '  STATE:', 'GO_TO_POINT'

            if (self.current_task == GO_TO_POINT_A):
                print '  Current_Task:', 'GO_TO_POINT_A'
                #print '  Head Joints:', self.kinematics.Jlist                
                #Q_des, command_result = self.traj_manager.head_trajectory_look_at_point()            
                Q_des, command_result = self.traj_manager.eye_trajectory_look_at_point()
                print '  Command Result:', command_result


                R_head_cur, p_head_cur = self.kinematics.get_6D_Head_Position(self.kinematics.Jlist)
                x_head_hat = np.array(R_head_cur)[:,0]
                print '             p_head_cur', p_head_cur
                print '             R_head_cur', np.array(R_head_cur)
                print '             x_head_hat', x_head_hat

                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_SOME_POINT):
                print '  Current_Task:', 'GO_TO_SOME_POINT'
                #print '  Head Joints:', self.kinematics.Jlist
                #Q_des, command_result = self.traj_manager.head_trajectory_look_at_point()            
                Q_des, command_result = self.traj_manager.head_trajectory_look_at_point()

                self.process_task_result(Q_des, command_result)                




        else:
            print "ERROR Not a valid state" 


    def loop(self):
        while not rospy.is_shutdown():
            self.task_logic()
            #self.behavior_logic()
            self.state_logic()          
            for i in range(0, 10): # 
                # send message
                self.rate.sleep()   #rospy.sleep(1/500.0);    
                # Will sleep for a total of 0.02 seconds --> 50Hz
            self.joint_publisher.publish_joints()



if __name__ == '__main__':
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()