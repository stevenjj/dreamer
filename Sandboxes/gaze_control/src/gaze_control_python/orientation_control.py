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

from detected_people_manager import *

from std_msgs.msg import Float32MultiArray

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

EYE_FOCUS = 107

GO_TO_POINT_HEAD_ONLY = 106
GO_TO_POINT_EYES_ONLY = 108
GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY = 109
GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY = 110
GO_TO_POINT_B = 111

# High Level Behavior List --- seems like this is another high level task list
DO_NOTHING = 200
ALWAYS_GAZE_AVERT = 201
ALWAYS_GAZE_FOLLOW = 202
MAKE_SQUARE = 203

MAKE_SQUARE_WITH_EYES_ONLY = 204
MAKE_SQUARE_WITH_HEAD_ONLY = 205
MAKE_SQUARE_WITH_PRIORITIZED_TASKS_EYES = 206
MAKE_SQUARE_WITH_PRIORITIZED_TASKS_HEAD = 207

class Dreamer_Head():
    command_once = False
    def __init__(self):
        self.behaviors = []
        self.tasks = [GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, GO_TO_POINT_HEAD_ONLY, EYE_FOCUS, NO_TASK, GAZE_AVERSION, TASK_HIERARCHY, CIRCULAR_HEAD_TRAJ, TRACK_PERSON, TRACK_MARKER]
        self.states = [IDLE, GO_TO_POINT, FOLLOWING_TRAJECTORY, FINISHED_TRAJECTORY, INTERRUPTED]

        self.current_state = IDLE
        self.current_task = NO_TASK

        self.kinematics = hk.Head_Kinematics() 
        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()

        self.traj_manager = Trajectory_Manager(self.kinematics)

        # ROS Loop details
        # Send control messages at 500hz. 10 messages to send per ROS loop so this node operates at 50Hz
        self.node_rate = 1/(500)
        self.rate = rospy.Rate(1000) 
        self.ROS_start_time = rospy.Time.now().to_sec()
        self.ROS_current_time = rospy.Time.now().to_sec()

        # Task Manager
        #self.task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
        self.task_list = [NO_TASK]        
        self.current_task_index = 0
        self.task_commanded = False
        self.current_task = self.task_list[self.current_task_index]

        self.task_params = []

        # Behavior Manager
        self.behavior_list =[MAKE_SQUARE_WITH_HEAD_ONLY, MAKE_SQUARE_WITH_EYES_ONLY, MAKE_SQUARE, MAKE_SQUARE_WITH_PRIORITIZED_TASKS_EYES,  MAKE_SQUARE_WITH_PRIORITIZED_TASKS_HEAD, DO_NOTHING] #[MAKE_SQUARE_WITH_EYES_ONLY, MAKE_SQUARE, DO_NOTHING]
        self.current_behavior_index = 0
        self.behavior_commanded = False
        self.behavior_task = self.behavior_list[0]

        # If behavior cycle, we will demonstrate all behaviors.
        self.behavior_cycle = False


        # Eye Focal Point initialize to not focused
        self.eyes_focused = False
        self.current_eye_focal_point = np.array([1,0,0])


        # Initialize desired gaze point location
        self.desired_gaze_point_location = np.array([1.0, 0.0, self.kinematics.l1])


        self.people_manager = Detected_People_Manager()
        self.track_marker_pos = np.array([1.0, 0, 0]) #x = 0, y = 0, z =0
        self.track_human_pos = np.array([1.0, 0, 0]) #x = 0, y = 0, z =0        


        self.focus_length_pub = rospy.Publisher('setArrLength', Float32MultiArray, queue_size=1)

        # READ Current Joint Positions
        # Otherwise send HOME command

    def callback_change_state(self, string):
        #specifying subscriber callback via callback class_instance.method_name
        return 

    def callback_marker_tracker(self, string):
        # change behavior to track marker
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


        # TASK GO TO POINT Using Eyes Only
        if ((self.current_task == GO_TO_POINT_EYES_ONLY) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time

            xyz_gaze_loc = self.task_params[self.current_task_index][0]
            movement_duration = self.task_params[self.current_task_index][1]

            self.traj_manager.specify_gaze_point(start_time, xyz_gaze_loc, movement_duration)  

            self.task_commanded = True
          
        # TASK GO TO POINT Using Head Only
        elif ((self.current_task == GO_TO_POINT_HEAD_ONLY) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time

            xyz_gaze_loc = self.task_params[self.current_task_index][0]
            movement_duration = self.task_params[self.current_task_index][1]

            self.traj_manager.specify_gaze_point(start_time, xyz_gaze_loc, movement_duration)  
            self.task_commanded = True

        # TASK GO TO POINT Using Head Only
        elif ((self.current_task == GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time

            head_xyz_gaze_loc = self.task_params[self.current_task_index][0]
            eye_xyz_gaze_loc = self.task_params[self.current_task_index][1]            
            movement_duration = self.task_params[self.current_task_index][2]

            self.traj_manager.specify_head_eye_gaze_point(start_time, head_xyz_gaze_loc, eye_xyz_gaze_loc, movement_duration)  

            self.task_commanded = True

        # TASK GO TO POINT Using Head Only
        elif ((self.current_task == GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time

            head_xyz_gaze_loc = self.task_params[self.current_task_index][0]
            eye_xyz_gaze_loc = self.task_params[self.current_task_index][1]            
            movement_duration = self.task_params[self.current_task_index][2]

            self.traj_manager.specify_head_eye_gaze_point(start_time, head_xyz_gaze_loc, eye_xyz_gaze_loc, movement_duration)  

            self.task_commanded = True


        elif ((self.current_task == EYE_FOCUS) and (self.task_commanded == False)):
            self.current_state = GO_TO_POINT
            start_time = self.ROS_current_time
            movement_duration = 1
            # SPECIFY TRAJ_MANAGER COMMAND self.traj_manager.specify_gaze_point(start_time, Q_cur, xyz_gaze_loc, movement_duration)  
            self.task_commanded = True

 

# GO_TO_POINT_HEAD_ONLY
# GO_TO_POINT_EYES_ONLY

# behavior creates task list and task parameters
# task list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY]
# task_params = [(param), (param), (param), (param)]

#    def go_to_point_params(self, desired_gaze_point_location, move_duration):
#        self.desired_gaze_point_location = desired_gaze_point_location
#       

    def set_go_to_point_params(self, gaze_location, move_duration):
        params = (gaze_location, move_duration)
        return params

    def set_prioritized_go_to_point_params(self, head_gaze_location, eye_gaze_location, move_duration):
        params = (head_gaze_location, eye_gaze_location, move_duration)
        return params        

    def behavior_logic(self):
        if ((self.behavior_task == MAKE_SQUARE) and self.behavior_commanded == False):
            #task_list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
            task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, NO_TASK]            
            '''task_list = [GO_TO_POINT_EYES_ONLY, 
                         GO_TO_POINT_HEAD_ONLY, 
                         GO_TO_POINT_EYES_ONLY, 
                         GO_TO_POINT_HEAD_ONLY, 
                         GO_TO_POINT_EYES_ONLY, 
                         GO_TO_POINT_HEAD_ONLY, 
                         NO_TASK]
            '''            
            task_params = []
            task_params.append( self.set_go_to_point_params(np.array( [0.6, 0.25, self.kinematics.l1+0.05]),      1) )
            task_params.append( self.set_go_to_point_params(np.array( [0.6, 0.25, self.kinematics.l1-0.1]),      1) )
            task_params.append( self.set_go_to_point_params(np.array( [0.6, -0.25, self.kinematics.l1-0.1]),     1) )
            task_params.append( self.set_go_to_point_params(np.array( [0.6, -0.25, self.kinematics.l1+0.05]),     1) )                       
            task_params.append( self.set_go_to_point_params(np.array( [0.6, 0.25, self.kinematics.l1+0.05]),      1) )
            task_params.append( self.set_go_to_point_params(np.array( [0.6, 0.25, self.kinematics.l1-0.1]),      1) )                   

            # Initialize task parameters
            self.task_list = task_list
            self.task_params = task_params                      

            self.current_task_index = 0
            self.task_commanded = False
            self.current_task = self.task_list[self.current_task_index]            

            self.behavior_commanded = True
            print "HELLO ONCE!"

        if ((self.behavior_task == MAKE_SQUARE_WITH_HEAD_ONLY) and self.behavior_commanded == False):
            #task_list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
            #task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, NO_TASK]            
            task_list = [GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         NO_TASK]            

            task_params = []
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1+0.3]),  np.array([6, 0.0, self.kinematics.l1]),       1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1-0.3]),   np.array([6, 0.0, self.kinematics.l1]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, -0.15, self.kinematics.l1-0.3]),  np.array([6, 0.0, self.kinematics.l1]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, -0.15, self.kinematics.l1+0.3]), np.array([6, 0.0, self.kinematics.l1]),      1) )                       
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1+0.3]),  np.array([6, 0.0, self.kinematics.l1]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1-0.3]),   np.array([6, 0.0, self.kinematics.l1]),      1) )                   

            # Initialize task parameters
            self.task_list = task_list
            self.task_params = task_params                      

            self.current_task_index = 0
            self.task_commanded = False
            self.current_task = self.task_list[self.current_task_index]            

            self.behavior_commanded = True
            print "HELLO ONCE!"


        if ((self.behavior_task == MAKE_SQUARE_WITH_EYES_ONLY) and self.behavior_commanded == False):
            #task_list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
            #task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, NO_TASK]            
            task_list = [GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         NO_TASK]            

            task_params = []
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, -0.15, self.kinematics.l1-0.2]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, -0.15, self.kinematics.l1+0.2]),     1) )                       
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )                   

            # Initialize task parameters
            self.task_list = task_list
            self.task_params = task_params                      

            self.current_task_index = 0
            self.task_commanded = False
            self.current_task = self.task_list[self.current_task_index]            

            self.behavior_commanded = True
            print "HELLO ONCE!"

        elif ((self.behavior_task == MAKE_SQUARE_WITH_PRIORITIZED_TASKS_EYES) and self.behavior_commanded == False):
            #task_list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
            #task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, NO_TASK]            
            task_list = [GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY,
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, 
                         NO_TASK]            

            task_params = []
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1+0.2]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      0.2) )            
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1+0.2]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1-0.2]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, -0.15, self.kinematics.l1-0.2]), np.array( [0.6, -0.15, self.kinematics.l1-0.2]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, -0.15, self.kinematics.l1+0.2]), np.array( [0.6, -0.15, self.kinematics.l1+0.2]),     1) )                       
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1+0.2]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1-0.2]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )                   

            # Initialize task parameters
            self.task_list = task_list
            self.task_params = task_params                      

            self.current_task_index = 0
            self.task_commanded = False
            self.current_task = self.task_list[self.current_task_index]            

            self.behavior_commanded = True

        elif ((self.behavior_task == MAKE_SQUARE_WITH_PRIORITIZED_TASKS_HEAD) and self.behavior_commanded == False):
            #task_list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
            #task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, NO_TASK]            
            task_list = [GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY,
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, 
                         NO_TASK]            

            task_params = []
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1+0.2]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )            
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1+0.2]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1-0.2]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, -0.15, self.kinematics.l1-0.2]), np.array( [0.6, -0.15, self.kinematics.l1-0.2]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, -0.15, self.kinematics.l1+0.2]), np.array( [0.6, -0.15, self.kinematics.l1+0.2]),     1) )                       
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1+0.2]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.6, 0.15, self.kinematics.l1-0.2]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )                   

            # Initialize task parameters
            self.task_list = task_list
            self.task_params = task_params                      

            self.current_task_index = 0
            self.task_commanded = False
            self.current_task = self.task_list[self.current_task_index]            

            self.behavior_commanded = True            



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

            if (self.current_task == GO_TO_POINT_EYES_ONLY):
                print '  Current_Task:', 'GO_TO_POINT_EYES_ONLY'
                Q_des, command_result = self.traj_manager.eye_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_POINT_HEAD_ONLY):
                print '  Current_Task:', 'GO_TO_POINT_HEAD_ONLY'
                Q_des, command_result = self.traj_manager.head_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY):
                print '  Current_Task:', 'GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY'
                Q_des, command_result = self.traj_manager.fixed_head_eye_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY):
                print '  Current_Task:', 'GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY'
                Q_des, command_result = self.traj_manager.fixed_eye_head_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)


        else:
            print "ERROR Not a valid state" 


    def publish_focus_length(self):
        msg = Float32MultiArray()
        common_length = 0.6
#        focus_lengths = [common_length, common_length, common_length + self.kinematics.l2]
 #       msg.data = focus_lengths
        msg.data.append(self.traj_manager.current_focus_length[self.traj_manager.LE])                
        msg.data.append(self.traj_manager.current_focus_length[self.traj_manager.RE])
        msg.data.append(self.traj_manager.current_focus_length[self.traj_manager.H])
        self.focus_length_pub.publish(msg)

    def loop(self):
        while not rospy.is_shutdown():
            print ''
            self.people_manager.loop()
            self.behavior_logic()            
            self.task_logic()
            self.state_logic()          
            #for i in range(0, 10): # 
                # send message
                #self.rate.sleep()   #rospy.sleep(1/500.0);    
                # Will sleep for a total of 0.02 seconds --> 50Hz
            self.joint_publisher.publish_joints()
            self.publish_focus_length()
            self.rate.sleep()
            print ''


if __name__ == '__main__':
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()