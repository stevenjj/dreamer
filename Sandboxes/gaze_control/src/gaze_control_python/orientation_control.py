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
TRACK_PERSON = 208

class Dreamer_Head():
    command_once = False
    def __init__(self):
        self.behaviors = []
        self.tasks = [GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, GO_TO_POINT_HEAD_ONLY, EYE_FOCUS, NO_TASK, GAZE_AVERSION, TASK_HIERARCHY, CIRCULAR_HEAD_TRAJ]
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
        self.behavior_list =[TRACK_PERSON, MAKE_SQUARE_WITH_HEAD_ONLY, MAKE_SQUARE_WITH_EYES_ONLY, MAKE_SQUARE, MAKE_SQUARE_WITH_PRIORITIZED_TASKS_EYES,  MAKE_SQUARE_WITH_PRIORITIZED_TASKS_HEAD, DO_NOTHING] #[MAKE_SQUARE_WITH_EYES_ONLY, MAKE_SQUARE, DO_NOTHING]
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
        self.track_marker_pos = np.array([1.0, 0, 0]) #x = 1, y = 0, z =0
        
        self.track_human_pos = np.array([1.0, 0, self.kinematics.l1]) #x = 2, y = 0, z =0        
        self.track_prev_human_pos = np.array([1.0, 0, self.kinematics.l1])
        self.track_human_pos_head = np.array([1.0, 0, self.kinematics.l1])


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
            if val >= 0.9*jmax:
                print '    MAX Software Joint HIT! for joint', joint_name
                return 0.9*jmax
            elif (val <= 0.9*jmin):
                print '    MIN Software Joint HIT! for joint', joint_name                
                return 0.9*jmin
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
        if (self.current_task_index < len(self.task_list)):
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

        elif ((self.behavior_task == TRACK_PERSON) and self.behavior_commanded == False) :
            print 'behavior is to track person'
            #task_list = [GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, GO_TO_POINT_HEAD_ONLY, NO_TASK]
            #task_list = [GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, GO_TO_POINT_EYES_ONLY, NO_TASK]            

            if (len(self.people_manager.list_of_people) > 0):
                

                self.track_human_pos = self.identify_closest_person_from_gaze_focus()#self.people_manager.list_of_people[0].eye_position


            else:
                self.track_human_pos = np.array([1,0,self.kinematics.l1])

            if (self.track_person_condition() == False):
                return

            task_list = []
            task_params = []

            duration = 1.0

            # angle_between_eyes_and_head:
            eye_foc_vec = self.track_human_pos# np.array([self.track_human_pos[0], self.track_human_pos[1], 0])
            head_foc_vec = self.track_human_pos_head #np.array([self.track_human_pos_head[0], self.track_human_pos_head[1], 0])
            dp = eye_foc_vec.dot(head_foc_vec)
            eye_foc_vec_norm = np.linalg.norm( eye_foc_vec )
            head_foc_vec_norm = np.linalg.norm( head_foc_vec )
            phi_eye_head = np.arccos(eye_foc_vec.dot(head_foc_vec) / eye_foc_vec_norm / head_foc_vec_norm)

            phi_eye_head_deg = phi_eye_head*180.0/np.pi


            print 'EYE FOCUS:', eye_foc_vec
            print 'HEAD_FOCUS:', head_foc_vec
            print '         Angle between Head and EYE', phi_eye_head_deg


            sample_max_vel = 1
            dx_total = np.linalg.norm(self.track_human_pos - self.track_prev_human_pos) 
            if  (dx_total < 0.3) and (phi_eye_head_deg < (12.0)):
                #duration = (15/8.0/sample_max_vel)*dx_total # 0.15
                duration = 0.15
                # if eyes are not too far apart. Use head as main priority
                if(self.are_eyes_focused()):
                    task_list = [GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY, NO_TASK] 
                else:
                    task_list = [GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, NO_TASK]                     

                task_params.append( self.set_prioritized_go_to_point_params( self.track_human_pos_head, self.track_human_pos,  duration) )            
            else:
                self.track_human_pos_head = self.track_human_pos
                #duration = (15/8.0/sample_max_vel)*dx_total # 0.3                
                duration = 0.5
                task_list = [GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY, NO_TASK] 
                task_params.append( self.set_prioritized_go_to_point_params( self.track_human_pos, self.track_human_pos,  duration) )            



            # Initialize task parameters
            self.task_list = task_list
            self.task_params = task_params                      

            self.current_task_index = 0
            self.task_commanded = False
            self.current_task = self.task_list[self.current_task_index]            
            self.behavior_commanded = True
            


        return 

    def identify_closest_person_from_gaze_focus(self):
        #R_head, p_head = self.kinematics.get_6D_Head_Position(self.kinematics.Jlist)
        #x_head_hat = np.array(R_head)[:,0]        

        #h_fl = self.traj_manager.current_focus_length[self.traj_manager.H]

        #head_focus_vec = x_head_hat*h_fl
        #head_focus_vec = self.track_human_pos_head
        head_focus_vec = np.array([0,0,0])

        min_dist = 1000.0
        person_index = 0
        for i in range(0, len(self.people_manager.list_of_people)):
            person_eye_pos = self.people_manager.list_of_people[i].eye_position
            distance = np.linalg.norm( head_focus_vec - person_eye_pos )
            if (distance) < min_dist:
                person_index = i
                min_dist = distance

        return self.people_manager.list_of_people[person_index].eye_position               

    def track_person_condition(self):
        (x,y,z) = self.track_human_pos[0], self.track_human_pos[1], self.track_human_pos[2] 
        if ( ((z < -0.2) or (z > 0.5)) and ((x < 0.2) or (x > 3)) ):
            return False

        x_hat = np.array([1,0,0])
        dp = (self.track_human_pos.dot(np.array([1,0,0])))
        x_hat_norm = np.linalg.norm( x_hat )
        xyz_person_norm = np.linalg.norm( self.track_human_pos )

        phi = np.arccos(x_hat.dot(self.track_human_pos) / xyz_person_norm / x_hat_norm)

        if ((dp > 0) and (phi < (np.pi/4.0))):
            return True
        else:
            return False

    def are_eyes_focused(self):
        R_right_eye, p_right_eye = self.kinematics.get_6D_Right_Eye_Position(self.kinematics.Jlist)
        R_left_eye, p_left_eye = self.kinematics.get_6D_Left_Eye_Position(self.kinematics.Jlist)
        x_right_eye_hat = np.array(R_right_eye)[:,0]        
        x_left_eye_hat = np.array(R_left_eye)[:,0]

        re_fl = self.traj_manager.current_focus_length[self.traj_manager.RE]
        le_fl = self.traj_manager.current_focus_length[self.traj_manager.LE]        

        thresh = 0.2

        if(np.linalg.norm(x_right_eye_hat*re_fl - x_left_eye_hat*le_fl)) < thresh:
            return True
        else:
            return False




    def process_task_result(self, Q_des, command_result):
        self.update_head_joints(Q_des)
        if (command_result == True):
            self.current_state = IDLE
            self.task_commanded = False
            self.next_task()
            if (self.behavior_task == TRACK_PERSON):
                self.track_prev_human_pos = self.track_human_pos 
                self.behavior_commanded = False

    def state_logic(self):
        if (self.current_state == IDLE):
            print '  STATE:', 'Idle'
            if self.current_task == NO_TASK:
                print '  Current_Task:', 'NO Task'
                print '  Current Task Index:', self.current_task_index

        elif (self.current_state == GO_TO_POINT):
            #print '  STATE:', 'GO_TO_POINT'

            if (self.current_task == GO_TO_POINT_EYES_ONLY):
                print '  Current_Task:', 'GO_TO_POINT_EYES_ONLY'
                Q_des, command_result = self.traj_manager.eye_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_POINT_HEAD_ONLY):
                print '  Current_Task:', 'GO_TO_POINT_HEAD_ONLY'
                Q_des, command_result = self.traj_manager.head_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY):
                #print '  Current_Task:', 'GO_TO_POINT_USING_EYES_WITH_HEAD_MAIN_PRIORITY'
                Q_des, command_result = self.traj_manager.fixed_head_eye_trajectory_look_at_point()
                self.process_task_result(Q_des, command_result)

            elif (self.current_task == GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY):
                #print '  Current_Task:', 'GO_TO_POINT_USING_HEAD_WITH_EYES_MAIN_PRIORITY'
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


if __name__ == '__main__':
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()