#!/usr/bin/env python
import rospy

# Math Dependencies
import numpy as np
import modern_robotics as mr
import head_kinematics as hk
import util_quat as quat

from detected_people_manager import *
from gaze_focus_states import *
from dreamer_controller import *

import min_jerk_single as single
from min_jerk_coordinates import *

import random

# ROS 
import dreamer_joint_publisher
from gaze_control.srv import HeadJointCmd, HeadJointCmdRequest
from gaze_control.srv import RunProgram, RunProgramRequest
from std_msgs.msg import Int8
import tf

from GUI_params import *

# Program Constants
JOINT_LIM_BOUND = 0.9 #between 0 to 1.0

# Rate Constants
NODE_RATE = 20#10 # Update rate of this node in Hz

LOW_LEVEL_FREQ = 550



CMD_RATE = 20 # Update rate for publishing joint positions to client

SEND_RATE = 20 # Trusted Rate of sending
PRINT_RATE = 10 # Print rate for Debugging

# Time Constants
WAIT_TIME_GO_HOME = 3 # in seconds

# State List
STATE_IDLE = 0
STATE_GO_TO_POINT = 1
STATE_GO_HOME = 2

STATE_ID_TO_STRING = {STATE_IDLE: "STATE_IDLE",
                      STATE_GO_TO_POINT: "STATE_GO_TO_POINT",
                      STATE_GO_HOME: "STATE_GO_HOME"}

# Task List
TASK_NO_TASK = 100
TASK_GO_TO_POINT_HEAD_PRIORITY    = 101
TASK_GO_TO_POINT_EYE_PRIORITY     = 102
TASK_TRACK_PERSON_HEAD       = 103
TASK_TRACK_PERSON_EYES       = 104
TASK_TRACK_PERSON_BEST       = 105
TASK_AVOID_NEAR_PERSON = 106
TASK_GO_TO_WAYPOINTS_EYE_PRIORITY = 107
TASK_FOLLOW_WAYPOINTS = 108

TASK_ID_TO_STRING = {TASK_NO_TASK: "TASK_NO_TASK",
                     TASK_GO_TO_POINT_HEAD_PRIORITY: "TASK_GO_TO_POINT_HEAD_PRIORITY",
                     TASK_GO_TO_POINT_EYE_PRIORITY: "TASK_GO_TO_POINT_EYE_PRIORITY",
                     TASK_TRACK_PERSON_HEAD: "TASK_TRACK_PERSON_HEAD",
                     TASK_TRACK_PERSON_EYES: "TASK_TRACK_PERSON_EYES",         
                     TASK_AVOID_NEAR_PERSON: "TASK_AVOID_NEAR_PERSON",
                     TASK_TRACK_PERSON_BEST: "TASK_TRACK_PERSON_BEST",            
                     TASK_GO_TO_WAYPOINTS_EYE_PRIORITY: "TASK_GO_TO_WAYPOINTS_EYE_PRIORITY",
                     TASK_FOLLOW_WAYPOINTS: "TASK_FOLLOW_WAYPOINTS" }

# Behavior List
BEHAVIOR_NO_BEHAVIOR = 200
BEHAVIOR_DO_SQUARE_FIXED_EYES = 201
BEHAVIOR_DO_SQUARE_FIXED_HEAD = 202
BEHAVIOR_TRACK_NEAR_PERSON = 203
BEHAVIOR_TRACK_NEAR_PERSON_EYES = 204
BEHAVIOR_TRACK_NEAR_PERSON_BEST = 205
BEHAVIOR_AVOID_NEAR_PERSON = 206
BEHAVIOR_FOLLOW_WAYPOINTS = 207
BEHAVIOR_FOLLOW_CIRCLE = 208
BEHAVIOR_ID_TO_STRING = {BEHAVIOR_NO_BEHAVIOR: "BEHAVIOR_NO_BEHAVIOR",
                         BEHAVIOR_DO_SQUARE_FIXED_EYES: "BEHAVIOR_DO_SQUARE_FIXED_EYES",
                         BEHAVIOR_DO_SQUARE_FIXED_HEAD: "BEHAVIOR_DO_SQUARE_FIXED_HEAD",
                         BEHAVIOR_TRACK_NEAR_PERSON: "BEHAVIOR_TRACK_NEAR_PERSON",
                         BEHAVIOR_TRACK_NEAR_PERSON_EYES: "BEHAVIOR_TRACK_NEAR_PERSON_EYES",
                         BEHAVIOR_TRACK_NEAR_PERSON_BEST: "BEHAVIOR_TRACK_NEAR_PERSON_BEST",                         
                         BEHAVIOR_AVOID_NEAR_PERSON: "BEHAVIOR_AVOID_NEAR_PERSON"  ,
                         BEHAVIOR_FOLLOW_WAYPOINTS: "BEHAVIOR_FOLLOW_WAYPOINTS"    ,                   
                         BEHAVIOR_FOLLOW_CIRCLE: "BEHAVIOR_FOLLOW_CIRCLE"                       
                    }



from program_constants import *

#-----------------------------------------------
# ROS Client for Low Level control 
def setup_ctrl_deq_append():
    #rospy.wait_for_service('ctrl_deq_append')
    ctrl_deq_append = None
    try:
        ctrl_deq_append = rospy.ServiceProxy('ctrl_deq_append', HeadJointCmd)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None
    else:
        return ctrl_deq_append

def setup_run_program():
    #rospy.wait_for_service('run_gaze_program')
    try:
        run_gaze_program = rospy.ServiceProxy('run_gaze_program', RunProgram)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None
    else:
        return run_gaze_program
#-----------------------------------------------




class Dreamer_Head():
    def __init__(self):
        # Setup Robot Kinematics
        self.kinematics = hk.Head_Kinematics() 

        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()
        self.GUI_listener    = rospy.Subscriber(GUI_CMD_TOPIC, Int8, self.gui_callback)

        self.low_level_control = False

        if self.low_level_control:
            self.ctrl_deq_append = setup_ctrl_deq_append()
            self.run_gaze_program = setup_run_program()
        else:
            self.ctrl_deq_append, self.run_gaze_program = None, None

        # Node Rates
        # ROS Rate
        self.rate = rospy.Rate(NODE_RATE) 
        # Command Rate
        self.CMD_rate = CMD_RATE # rate to send commands to low levelin Hz
        # Print Rate
        self.print_rate = PRINT_RATE

        # GUI Commands
        self.gui_command = NO_COMMAND
        self.gui_command_string = NO_COMMAND_STRING


        # Class Objects
        
        self.people_manager = Detected_People_Manager()
        self.gaze_focus_states = Gaze_Focus_States(self.kinematics)
        self.controller_manager = Controller(self.kinematics, self.gaze_focus_states, self.people_manager, self.joint_publisher)       

        # Behaviors, States and Tasks
        self.states = [STATE_IDLE, STATE_GO_TO_POINT, STATE_GO_HOME]
        self.behaviors =[BEHAVIOR_NO_BEHAVIOR, BEHAVIOR_DO_SQUARE_FIXED_EYES]
        self.tasks = [TASK_NO_TASK]

        # Current State, Behavior, Task
        self.current_state    = STATE_IDLE
        self.current_behavior = BEHAVIOR_NO_BEHAVIOR
        self.current_task     = TASK_NO_TASK

        # Task Variables
        self.task_list = []
        self.task_params = []
        self.current_task_index = 0                


        # Time Keeping Variables
        self.ROS_start_time = rospy.get_time()
        self.ROS_current_time = self.ROS_start_time        
        self.relative_time = self.ROS_current_time - self.ROS_start_time
        self.time_since_last_cmd_sent = self.ROS_start_time
        self.time_since_last_print = self.ROS_start_time
        self.time_since_last_loop = self.ROS_start_time        

        self.interval= 1.0/NODE_RATE

        self.gui_command_execute_time = self.ROS_start_time                

        self.dt = 1.0/NODE_RATE


        self.cmd_rate_measured = 1/self.dt        

        # Custom Wait Times
        self.wait_time_go_home = WAIT_TIME_GO_HOME

        # Semaphores
        self.gui_command_executing = False
        self.behavior_commanded = False
        self.task_commanded = False
        
        # For my slow af computer
        self.publish_to_low_level = False
        self.low_level_fifo = []
        self.calculate_low_level = False

    #-------------------------------------------------------------
    # Joint and Gaze Focus Update
    
    # Function: Sets the variables for the joints
    def update_head_joints(self, head_joint_list):
        self.kinematics.Jlist = head_joint_list
        self.gaze_focus_states.update_gaze_focus_states(self.interval)

        # Function: Bounds the joint movement
        # Note that this function is hidden from all other functions, only needed within this function
        def joint_cmd_bound(val, joint_name, jmax, jmin):
            if val >= JOINT_LIM_BOUND*jmax:
                print '    MAX Software Joint HIT! for joint', joint_name
                return JOINT_LIM_BOUND*jmax
            elif (val <= JOINT_LIM_BOUND*jmin):
                print '    MIN Software Joint HIT! for joint', joint_name                
                return JOINT_LIM_BOUND*jmin
            else:
                return val

        for i in range(0, len(head_joint_list)):
            command = head_joint_list[i]
            joint_name = self.kinematics.Jindex_to_names[i]
            joint_max_val = self.joint_publisher.free_joints[joint_name]['max']
            joint_min_val = self.joint_publisher.free_joints[joint_name]['min']
            joint_val = joint_cmd_bound(command, joint_name, joint_max_val, joint_min_val)
            self.joint_publisher.free_joints[joint_name]['position'] = joint_val
            self.kinematics.Jlist[i] = joint_val

    # Function: Update time of the head class
    # Updates: dt - time since last update
    #          current 'system' time
    #          relative time since ROS initialization
    def update_time(self):
        current_time = rospy.get_time()
        self.dt = current_time - self.ROS_current_time
        self.ROS_current_time = current_time
        self.relative_time =  self.ROS_current_time - self.ROS_start_time                 
        return

    # ----------------------------------------------------------------
    # Low Level Functions 
    def enable_low_level(self):
        self.low_level_control = True
        self.ctrl_deq_append = setup_ctrl_deq_append()
        self.run_gaze_program = setup_run_program()

    def disable_low_level(self):
        self.low_level_control = False        
        self.ctrl_deq_append = None
        self.run_gaze_program = None        

    def prepare_joint_command(self):
        # Send 7 joints
        joints = range(7)
        joint_rads = self.kinematics.Jlist[0:7]
        return joints, joint_rads

    def send_low_level_commands(self):
        # Prepare Latest Joint List
        joint_map, joint_val = self.prepare_joint_command()

        hjc = HeadJointCmdRequest()
        hjc.numCtrlSteps.data = LOW_LEVEL_FREQ*self.interval 

        joints = joint_map 
        rads = joint_val

        print rads

        hjc.joint_mapping.data = joints
        hjc.q_cmd_radians.data = rads

        # Send latest joint list
        try:
            self.ctrl_deq_append(hjc)
        except:
            print 'failed to send message'
            print 'ctrl_deq_append must not be initialized'
        return

    def send_go_home_low_level_command(self):
        # Prepare Latest Joint List
        joint_map, joint_val = range(7), np.zeros(7)

        hjc = HeadJointCmdRequest()
        hjc.numCtrlSteps.data = LOW_LEVEL_FREQ*self.wait_time_go_home 

        joints = joint_map
        rads = joint_val

        print rads

        hjc.joint_mapping.data = joints
        hjc.q_cmd_radians.data = rads

        # Send latest joint list
        try:
            self.ctrl_deq_append(hjc)
        except:
            print 'Failed to go home. ctrl_deq_append must not be initialized'
            rospy.sleep(1)
            self.gui_command_executing = False
            self.current_state = STATE_IDLE
        return



# ----------------------- Commands for Manual Publish of joints -------------------------
    # Function: Saves joint positions to a fifo for later publishing
    # Inputs: None
    # Outputs: None
    def save_low_level_commands(self):
        joint_map, joint_val = self.prepare_joint_command()
        joint_val = joint_val
        hjc = HeadJointCmdRequest()
        hjc.numCtrlSteps.data = LOW_LEVEL_FREQ*self.interval 

        joints = joint_map
        rads = joint_val

        hjc.joint_mapping.data = joints
        hjc.q_cmd_radians.data = rads
        self.low_level_fifo.append(hjc)
        return

    # Function: Publish saved joint positions from low level fifo to dreamer robot
    # Inputs: None
    # Outputs: None
    # Notes: Make sure not to publish commands too quickly or it'll skip some
    #        After completion will clear the fifo, disable low level, and turn off low level publishing
    def publish_low_level_commands(self):
        self.enable_low_level()
        print "Publishing Low Level Commands"
        i = 0
        while ( (not rospy.is_shutdown()) and (i < len(self.low_level_fifo))):
            self.update_time()
            self.interval = self.ROS_current_time - self.time_since_last_loop                       
            # Append commands at ~~ Hz to not overflow the fifo
            if (self.interval > (1.0/20.0)):
                try:
                    self.ctrl_deq_append(self.low_level_fifo[i])
                except:
                    print 'failed to send message'
                    print 'ctrl_deq_append must not be initialized'
                self.time_since_last_loop = self.ROS_current_time
                i+=1
        print "Low Level Publish Complete"
        self.disable_low_level()
        self.low_level_fifo = []
        self.publish_to_low_level = False
        return



    # Function: Calculates joints at a specified rate, saves them, and visualizes them in Rviz
    # Inputs: Rate in Hz of joint calculation
    # Outputs: None
    # Notes: In order for this to work, a time variable needed to be added to state_logic and fancy math
    #           for timing
    #        I calculate the number of terms in the fifo by multiplying the seconds by the rate/second
    #        Rviz will try to visualize the behavior at the specified rate
    def calculate_behavior_joints(self, rate):
        print "\nCalculating Joints"
        current_time = rospy.get_time()
        self.current_behavior = BEHAVIOR_FOLLOW_WAYPOINTS  
        time = 0
        while (self.current_behavior == BEHAVIOR_FOLLOW_WAYPOINTS):
            self.behavior_logic()
            self.task_logic()
            self.state_logic(time)
            self.save_low_level_commands()
            time += 1.0/rate

        self.calculate_low_level = False
        print "Took: ", rospy.get_time() - current_time, "s"
        print "Joint Calculation Complete\n"

        f = open('gaze_control_python/output.txt', 'w')
        f.write(str(self.low_level_fifo))
        print "Low Level Commands saved to output.txt"
        # Need to iterate through the low level fifo now
        print "\nVisualizing in rviz"

        loop = 0
        while(loop < len(self.low_level_fifo)):
            self.update_time()
            # print loop
            self.interval = self.ROS_current_time - self.time_since_last_loop              
            if self.interval > (1.0/rate):
                self.update_head_joints(self.low_level_fifo[loop].q_cmd_radians.data)
                self.joint_publisher.publish_joints()
                self.gaze_focus_states.publish_focus_length()
                loop += 1
                self.time_since_last_loop = self.ROS_current_time

        print "Visualization Complete"



    # -----------------------------------------------------------------
    # Function: Send Joint Commands to RVIZ
    #   and to low Level if possible @ the control loop frequency
    def send_command(self):      
        cmd_interval = self.ROS_current_time - self.time_since_last_cmd_sent

        # if (cmd_interval > (1.0/( float(self.CMD_rate))) ):   
            # Send Low Level Commands if LL Control is enabled
        if (self.low_level_control and 
            self.current_state != STATE_IDLE and 
            self.current_state != STATE_GO_HOME
            and (not self.publish_to_low_level)):

            print 'sending low level commands now'
            self.send_low_level_commands()

        elif(self.current_state != STATE_IDLE and
            self.current_state != STATE_GO_HOME):
            self.save_low_level_commands()

        self.cmd_rate_measured = 1.0/self.interval #1.0/self.dt #1.0/cmd_interval
        self.time_since_last_cmd_sent = rospy.get_time()

        # Send to Rviz
        self.joint_publisher.publish_joints()
        return


    # Function: Self-explanatory
    def reset_all_joints_to_home(self):
        self.kinematics.Jlist = np.zeros(7)            
        self.update_head_joints(self.kinematics.Jlist)
        self.gaze_focus_states.reset()  

    
    # ---------------------------------------------------------
    # GUI Callback Commands
    def gui_callback(self, msg):
        gui_command = msg.data
        if gui_command == LOW_LEVEL_OFF:
            self.gui_command = gui_command
            self.gui_command_string = LOW_LEVEL_OFF_STRING
            #---------------------------------------------
            # Disable Low Level
            self.disable_low_level()

        elif gui_command == LOW_LEVEL_ON:
            self.gui_command = gui_command
            self.gui_command_string = LOW_LEVEL_ON_STRING
            #---------------------------------------------  
            # Enable Low Level
            self.enable_low_level()

        elif gui_command == STATE_TO_IDLE:
            self.gui_command = gui_command
            self.gui_command_string = STATE_TO_IDLE_STRING
            #---------------------------------------------             
            # Change State to Idle
            self.current_state = STATE_IDLE                     
            self.reset_state_tasks_behaviors()

            # Clear low level fifo saved information
            self.publish_to_low_level = False
            self.low_level_fifo = []
        
        elif gui_command == GO_HOME:
            self.gui_command = gui_command
            self.gui_command_string = GO_HOME_STRING
            #---------------------------------------------  
            # Change State to Go Home              
            self.reset_state_tasks_behaviors()
            self.current_state = STATE_GO_HOME                   
            self.gui_command_execute_time = rospy.get_time()

        
        elif gui_command == DO_SQUARE_FIXED_EYES:  
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_FIXED_EYES_STRING
            #---------------------------------------------  
            # Change Current Behavior
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_DO_SQUARE_FIXED_EYES

        elif gui_command == DO_SQUARE_FIXED_HEAD:
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_FIXED_HEAD_STRING
            # Change Current Behavior
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_DO_SQUARE_FIXED_HEAD
        
        elif gui_command == TRACK_NEAR_PERSON:
            self.gui_command = gui_command
            self.gui_command_string = TRACK_NEAR_PERSON_STRING
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_TRACK_NEAR_PERSON

        elif gui_command == TRACK_NEAR_PERSON_EYES:
            self.gui_command = gui_command
            self.gui_command_string = TRACK_NEAR_PERSON_EYES_STRING
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_TRACK_NEAR_PERSON_EYES            

        elif gui_command == TRACK_NEAR_PERSON_BEST:
            self.gui_command = gui_command
            self.gui_command_string = TRACK_NEAR_PERSON_BEST_STRING
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_TRACK_NEAR_PERSON_BEST            

        elif gui_command == AVOID_NEAR_PERSON:
            self.gui_command = gui_command
            self.gui_command_string = AVOID_NEAR_PERSON_STRING
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_AVOID_NEAR_PERSON

        elif gui_command == DO_WAYPOINT_TRAJ:      
            self.gui_command = gui_command
            self.gui_command_string = DO_WAYPOINT_TRAJ_STRING
            self.reset_state_tasks_behaviors()            
            self.current_behavior = BEHAVIOR_FOLLOW_WAYPOINTS

        elif gui_command == CIRCLE_TRAJ:      
            self.gui_command = gui_command
            self.gui_command_string = CIRCLE_TRAJ_STRING
            self.reset_state_tasks_behaviors()            
            self.current_behavior = BEHAVIOR_FOLLOW_CIRCLE  

        elif gui_command == LOW_LEVEL_PUBLISH:
            self.gui_command = gui_command
            self.gui_command_string = LOW_LEVEL_PUBLISH_STRING
            print "Low Level Commands saved to output.txt"
            f = open('gaze_control_python/output.txt', 'w')
            f.write(str(self.low_level_fifo))
            self.publish_to_low_level = True # Variable used to stop standard low level publisher

        elif gui_command == CALC_BEHAV_AT_RATE:
            self.gui_command = gui_command
            self.gui_command_string = CALC_BEHAV_AT_RATE_STRING
            self.disable_low_level()
            self.publish_to_low_level = False
            self.low_level_fifo = []
            self.calculate_low_level = True

        
        else:
            print gui_command, "Invalid Command"


    # Print Statements
    def print_debug(self):
        print_interval = self.ROS_current_time - self.time_since_last_print
        if (print_interval > (1.0/( float(self.print_rate))) ):
            self.time_since_last_print = rospy.get_time()        
            print 'ROS time (sec)          :' , self.relative_time
            print '    State               :' , STATE_ID_TO_STRING[self.current_state]
            print '    Task                :' , TASK_ID_TO_STRING[self.current_task]
            print '    Behavior            :' , BEHAVIOR_ID_TO_STRING[self.current_behavior]
            print '    Low Level           :' , self.low_level_control 
            print '    GUI Command         :' , self.gui_command_string
            print '        Command Rate (Hz)   :' , self.cmd_rate_measured  
            print '        Print Rate (Hz)     :' , 1.0/print_interval
            print '        Node Rate (Hz)      :' , 1.0/self.interval #self.dt 
            print '        Node dt(s)          :' , self.interval #self.dt
            print '        Custom Sleep(Hz)    ;' , 1.0/self.interval

            print '    Semaphores'
            print "        GUI,   Behavior, Task CMD" 
            print "       ", self.gui_command_executing, " ", self.behavior_commanded, "   ", self.task_commanded

            print '     Joint List:'
            print '       ', self.kinematics.Jlist


            J0, J1, J2, J3, J4, J5, J6 = self.kinematics.Jlist
            print '    head pitch       ', J0+J3            
            print '    left eye pitch   ', J6
            print '    right eye pitch  ', J6
            # if (self.current_task == TASK_FOLLOW_WAYPOINTS):
            #     print '     MinJerk:'
            #     time = rospy.get_time() - self.BRANDON_TIME
            #     coordinate1 = self.task_params[self.current_task_index]
            #     x = coordinate1.get_position(time)[0]
            #     y = coordinate1.get_position(time)[1]
            #     z = coordinate1.get_position(time)[2]                        
            #     print '       (t, x, y, z):', (time, x, y, z)

            # print self.current_task_index
            # print len(self.task_list)
            # print self.task_params

            #self.gaze_focus_states.print_debug()
            #self.gaze_focus_states.print_debugxy()
            #self.controller_manager.print_debug()
            # self.people_manager.print_debug()


    

    # ---------------- Behavior and Task Logic ----------------

    #----------------------------------------------------------
    # Behavior Helper Functions

    # Function: Formats the data that is appended to task_params
    def set_prioritized_go_to_point_params(self, head_gaze_location, eye_gaze_location, move_duration):
        params = (head_gaze_location, eye_gaze_location, move_duration)
        return params       


    # Function: Update behavior variables
    #           Set up for the behavior to be executed
    #           behavior_commanded is now true
    def execute_behavior(self, task_list, task_params):
        # Update Task Variables
        self.task_list   = task_list
        self.task_params = task_params

        self.current_task_index = 0
        self.current_task = self.task_list[self.current_task_index]
        
        # Update Semaphores
        self.task_commanded     = False
        self.behavior_commanded = True


    # Function: Define behaviors and tasks within each behavior
    #           Behaviors are a wrapper for a set of tasks
    #   Logic:
    #   1. if-else statements will find which behavior was clicked:
    #       if a behavior was clicked, the gui_handler will reset the behavior_commanded semaphore 
    #       if there is a behavior in progress, exit function without doing anything
    #   2. task_list contains the name of the task for the behavior
    #   3. task_params contain the actual commands such as moving to a point
    #   4. Updates variables with execute_behavior with task_list and task_params
    def behavior_logic(self):
        if self.current_behavior == BEHAVIOR_NO_BEHAVIOR:
            self.task_list = [TASK_NO_TASK]
            self.task_params = []
            return

        # This behavior makes a square with the head while the eyes are pointing straight ahead
        # Redundant checking of behavior_commanded
        elif ((self.current_behavior == BEHAVIOR_DO_SQUARE_FIXED_EYES) and self.behavior_commanded == False):
            task_list = [TASK_GO_TO_POINT_EYE_PRIORITY for i in range(6)]
#            task_list = [TASK_GO_TO_POINT_HEAD_PRIORITY for i in range(6)]            
            task_params = []

            init_to_go_point = 3 # Take a longer time to go to the initial point
            duration = 2
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [1.2, 0.25, self.kinematics.l1+1.3]),  np.array([0.75, 0., self.kinematics.l1-0.]),      init_to_go_point) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [1.2, 1.25, self.kinematics.l1-1.3]),   np.array([0.75, 0., self.kinematics.l1-0.]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [1.2, -1.25, self.kinematics.l1-1.3]),  np.array([0.75, 0., self.kinematics.l1-0.]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [1.2, -1.25, self.kinematics.l1+1.3]), np.array([0.75, 0., self.kinematics.l1-0.]),      duration) )                       
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [1.2, 1.25, self.kinematics.l1+1.3]),  np.array([0.75, 0., self.kinematics.l1-0.]),      duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [1.2, 0.25, self.kinematics.l1-1.3]),   np.array([0.75, 0., self.kinematics.l1-0.]),     duration) )

            # # Fixed Eye Big Head Motion
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1+0.2]),  np.array([0.75, 0.0, self.kinematics.l1+0.0]),      init_to_go_point) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1-0.2]),   np.array([0.75, 0.0, self.kinematics.l1-0.0]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.55, self.kinematics.l1-0.2]),  np.array([0.75, -0.0, self.kinematics.l1-0.0]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.55, self.kinematics.l1+0.2]), np.array([0.75, -0.0, self.kinematics.l1+0.0]),      duration) )                       
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1+0.2]),  np.array([0.75, 0.0, self.kinematics.l1+0.0]),      duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1-0.2]),   np.array([0.75, 0.0, self.kinematics.l1-0.0]),     duration) )

            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1+0.2]),  np.array([0.75, 0.0, self.kinematics.l1+0.0]),      init_to_go_point) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1-0.2]),   np.array([0.75, 0.0, self.kinematics.l1-0.0]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.15, self.kinematics.l1-0.2]),  np.array([0.75, -0.0, self.kinematics.l1-0.0]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.15, self.kinematics.l1+0.2]), np.array([0.75, -0.0, self.kinematics.l1+0.0]),      duration) )                       
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1+0.2]),  np.array([0.75, 0.0, self.kinematics.l1+0.0]),      duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1-0.2]),   np.array([0.75, 0.0, self.kinematics.l1-0.0]),     duration) )


            # Square Eye Big Head Motion
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1+0.2]),  np.array([0.75, 0.05, self.kinematics.l1+0.05]),      init_to_go_point) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1-0.2]),   np.array([0.75, -0.05, self.kinematics.l1+0.05]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.55, self.kinematics.l1-0.2]),  np.array([0.75, -0.05, self.kinematics.l1-0.05]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.55, self.kinematics.l1+0.2]), np.array([0.75, 0.05, self.kinematics.l1-0.05]),      duration) )                       
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1+0.2]),  np.array([0.75, 0.05, self.kinematics.l1+0.05]),      duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1-0.2]),   np.array([0.75, -0.05, self.kinematics.l1+0.05]),     duration) )


            # Square Eye Bigger Head Motion
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1+0.3]),  np.array([0.75, 0.05, self.kinematics.l1+0.05]),      init_to_go_point) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1-0.3]),   np.array([0.75, -0.05, self.kinematics.l1+0.05]),     duration) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.55, self.kinematics.l1-0.3]),  np.array([0.75, -0.05, self.kinematics.l1-0.05]),     duration) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.55, self.kinematics.l1+0.3]), np.array([0.75, 0.05, self.kinematics.l1-0.05]),      duration) )                       
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1+0.3]),  np.array([0.75, 0.05, self.kinematics.l1+0.05]),      duration) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.55, self.kinematics.l1-0.3]),   np.array([0.75, -0.05, self.kinematics.l1+0.05]),     duration) )



            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1+0.2]),  np.array([0.75, 0.05, self.kinematics.l1+0.05]),      init_to_go_point) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1-0.2]),   np.array([0.75, -0.05, self.kinematics.l1+0.05]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.15, self.kinematics.l1-0.2]),  np.array([0.75, -0.05, self.kinematics.l1-0.05]),     duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, -0.15, self.kinematics.l1+0.2]), np.array([0.75, 0.05, self.kinematics.l1-0.05]),      duration) )                       
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1+0.2]),  np.array([0.75, 0.05, self.kinematics.l1+0.05]),      duration) )
            # task_params.append( self.set_prioritized_go_to_point_params( np.array( [0.7, 0.15, self.kinematics.l1-0.2]),   np.array([0.75, -0.05, self.kinematics.l1+0.05]),     duration) )

            self.execute_behavior(task_list, task_params)

        # This behavior makes a square with the eyes while the head points straight ahead
        elif ((self.current_behavior == BEHAVIOR_DO_SQUARE_FIXED_HEAD) and self.behavior_commanded == False):
#            task_list = [TASK_GO_TO_POINT_EYE_PRIORITY for i in range(6)]
            task_list = [TASK_GO_TO_POINT_HEAD_PRIORITY for i in range(6)]            
            task_params = []

            init_to_go_point = 3 # Take a longer time to go to the initial point
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.7, 0.15, self.kinematics.l1+0.2]),      init_to_go_point) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.7, 0.15, self.kinematics.l1-0.2]),      2) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.7, -0.15, self.kinematics.l1-0.2]),     2) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.7, -0.15, self.kinematics.l1+0.2]),     2) )                       
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.7, 0.15, self.kinematics.l1+0.2]),      2) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.7, 0.15, self.kinematics.l1-0.2]),      2) )                   
            #task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [-0.7, 0.55, self.kinematics.l1-0.2]),      1) )                               
            self.execute_behavior(task_list, task_params)

        # There are no defined task_params for tracking people because it is calculated at run-time
        elif ((self.current_behavior == BEHAVIOR_TRACK_NEAR_PERSON) and self.behavior_commanded == False):
            task_list = [TASK_TRACK_PERSON_HEAD]            
            task_params = []
            self.execute_behavior(task_list, task_params)

        elif ((self.current_behavior == BEHAVIOR_TRACK_NEAR_PERSON_EYES) and self.behavior_commanded == False):
            task_list = [TASK_TRACK_PERSON_EYES]            
            task_params = []
            self.execute_behavior(task_list, task_params)


        elif ((self.current_behavior == BEHAVIOR_TRACK_NEAR_PERSON_BEST) and self.behavior_commanded == False):
            task_list = [TASK_TRACK_PERSON_BEST]            
            task_params = []
            self.execute_behavior(task_list, task_params)


        elif ((self.current_behavior == BEHAVIOR_AVOID_NEAR_PERSON) and self.behavior_commanded == False):
            task_list = [TASK_AVOID_NEAR_PERSON]            
            movement_duration = 1.0 # Take 1. look away
            task_params = [ [movement_duration] ]

            self.execute_behavior(task_list, task_params)

        elif ((self.current_behavior == BEHAVIOR_FOLLOW_WAYPOINTS) and self.behavior_commanded == False):
            task_list = [TASK_GO_TO_POINT_HEAD_PRIORITY, TASK_FOLLOW_WAYPOINTS]
            task_params = []

            #piecewise_func_head, piecewise_func_eyes = eye_constant_head_square(0.8)#squares_task(0.8) #eye_constant_head_square(0.7)#hold_center_head_square(0.3) #squares_task(0.3)#squares_task(0.9) #hold_center_head_square(1.0)#squares_task(1.0) 
            piecewise_func_head, piecewise_func_eyes = squares_task(0.8) #eye_constant_head_square(0.7)#hold_center_head_square(0.3) #squares_task(0.3)#squares_task(0.9) #hold_center_head_square(1.0)#squares_task(1.0) 

            # Extract initial coordinates
            head_coord = piecewise_func_head.get_position(0)
            x_head = head_coord[0]
            y_head = head_coord[1]
            z_head = head_coord[2]   

            # Extract initial coordinates
            eyes_coord = piecewise_func_eyes.get_position(0)
            x_eyes = eyes_coord[0]
            y_eyes = eyes_coord[1]
            z_eyes = eyes_coord[2]            
            scale = 1.0

            # Go to the initial coordinates before executing minimum jerk
            duration = 4.0 * scale
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [x_head, y_head, z_head] ), np.array( [x_eyes, y_eyes, z_eyes] ), duration) )
            
            total_run_time = piecewise_func_head.total_run_time()
            task_params.append( (piecewise_func_head, piecewise_func_eyes, total_run_time) )
            self.execute_behavior(task_list, task_params)

        elif ((self.current_behavior == BEHAVIOR_FOLLOW_CIRCLE) and self.behavior_commanded == False):
            task_list = [TASK_GO_TO_POINT_HEAD_PRIORITY, TASK_FOLLOW_WAYPOINTS]
            task_params = []
            # Draw a circle behavior
            piecewise_func = circle_yz(1.0, 12.0)
            # Extract initial coordinates
            coord = piecewise_func.get_position(0)
            x = coord[0]
            y = coord[1]
            z = coord[2]       

            # Go to the initial coordinates before executing minimum jerk
            duration = 3.0
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [x, y, z] ), np.array( [x, y, z] ), duration) )
            
            total_run_time = piecewise_func.total_run_time()
            task_params.append( (piecewise_func, piecewise_func, total_run_time) )
            self.execute_behavior(task_list, task_params)
            
        return


    #----------------------------------------------------------

    # Function: Executes a single task based on the current index of the task_params
    # This function executes many times over the course of a behavior
    # Logic:
    #       1. Find what the current task is and if no task is currently in progress
    #       2. Save the ROS time
    #       3. Set up more parameters if applicable
    #       4. Set current state and task_commanded
    def task_logic(self):
        if self.current_task == TASK_NO_TASK:
            return

        # The following two tasks take points from the params and move eyes/head to those locations based on a minimum jerk
        elif ((self.current_task == TASK_GO_TO_POINT_EYE_PRIORITY) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT
            start_time = rospy.Time.now().to_sec()  #self.ROS_current_time
            head_xyz_gaze_loc = self.task_params[self.current_task_index][0]
            eye_xyz_gaze_loc = self.task_params[self.current_task_index][1]            
            movement_duration = self.task_params[self.current_task_index][2]

            self.controller_manager.specify_head_eye_gaze_point(start_time, head_xyz_gaze_loc, eye_xyz_gaze_loc, movement_duration)  
            self.task_commanded = True

        elif ((self.current_task == TASK_GO_TO_POINT_HEAD_PRIORITY) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT
            start_time = rospy.Time.now().to_sec()  #self.ROS_current_time
            head_xyz_gaze_loc = self.task_params[self.current_task_index][0]
            eye_xyz_gaze_loc = self.task_params[self.current_task_index][1]            
            movement_duration = self.task_params[self.current_task_index][2]

            self.controller_manager.specify_head_eye_gaze_point(start_time, head_xyz_gaze_loc, eye_xyz_gaze_loc, movement_duration)  
            self.task_commanded = True

        # The following three tasks don't have specifically defined movement 
        #    because they are based on tracking people
        elif ((self.current_task == TASK_TRACK_PERSON_HEAD) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT
            self.task_commanded = True


        elif ((self.current_task == TASK_TRACK_PERSON_EYES) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT
            self.task_commanded = True


        elif ((self.current_task == TASK_TRACK_PERSON_BEST) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT
            self.task_commanded = True


        elif ((self.current_task == TASK_AVOID_NEAR_PERSON) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT

            start_time = rospy.Time.now().to_sec()  #self.ROS_current_time
            movement_duration = self.task_params[self.current_task_index][0]            
            self.controller_manager.specify_avoid_person_params(start_time, movement_duration)
            self.task_commanded = True


        elif ((self.current_task == TASK_FOLLOW_WAYPOINTS) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT

            start_time = rospy.Time.now().to_sec()  #self.ROS_current_time
            piecewise_func_head = self.task_params[self.current_task_index][0]
            piecewise_func_eyes = self.task_params[self.current_task_index][1]
            total_run_time = self.task_params[self.current_task_index][2] 

            self.controller_manager.specify_follow_traj_params(start_time, total_run_time, piecewise_func_head, piecewise_func_eyes)
            self.task_commanded = True


        return


    # ---------------------------------------------------------
    # State Helper Functions

    # Function: Helper function that shifts the current task to the next one in the list if index not out of bounds
    def next_task(self):
        if (self.current_task_index < len(self.task_list)):
            self.current_task = self.task_list[self.current_task_index]

    # Function: Updates variables for head joints, moves on to next task if task is completed
    #           If all tasks are complete, reset
    # Inputs: Desired Joint Configuration, Whether or not the task is now complete
    # Returns: None
    def process_task_result(self, Q_des, command_result):
        self.update_head_joints(Q_des)
        if (command_result == True):
            self.task_commanded = False
            self.current_task_index += 1
            self.next_task()          

            if self.current_task_index == len(self.task_list):
                self.reset_state_tasks_behaviors()

    # Function: Resets all command variables in Dreamer to nothing commanded
    def reset_state_tasks_behaviors(self):
        self.current_state = STATE_IDLE
        self.current_task = TASK_NO_TASK 
        self.current_behavior = BEHAVIOR_NO_BEHAVIOR
        self.gui_command_executing = False
        self.behavior_commanded = False
        self.task_commanded = False
        self.publish_to_low_level = False # Variable used to stop standard low level publisher


    # Main State Machine
    #   State Machine computes current joint positions
    def state_logic(self, time = None):
        #--------------
        # STATE IDLE
        if (self.current_state == STATE_IDLE):
            return # Do Nothing

        #--------------
        # STATE GO TO POINT
        # Calls task from the controller_manager which returns a Q change and a result of success or failure of the command
        elif (self.current_state == STATE_GO_TO_POINT):
            if (self.current_task == TASK_GO_TO_POINT_EYE_PRIORITY):
                 Q_des, command_result = self.controller_manager.eye_priority_head_trajectory_look_at_point()
                 self.process_task_result(Q_des, command_result)

            elif (self.current_task == TASK_GO_TO_POINT_HEAD_PRIORITY):
                 Q_des, command_result = self.controller_manager.head_priority_eye_trajectory_look_at_point(time)                
                 self.process_task_result(Q_des, command_result)

            elif (self.current_task == TASK_TRACK_PERSON_HEAD):
                 Q_des, command_result = self.controller_manager.control_track_person(self.interval)
                 self.process_task_result(Q_des, command_result)                 
            elif (self.current_task == TASK_TRACK_PERSON_EYES):
                 Q_des, command_result = self.controller_manager.control_track_person_eyes_only(self.interval)
                 self.process_task_result(Q_des, command_result)                 

            elif (self.current_task == TASK_TRACK_PERSON_BEST):
                 Q_des, command_result = self.controller_manager.control_track_person_eye_priority(self.interval)
                 self.process_task_result(Q_des, command_result)                 

            elif (self.current_task == TASK_AVOID_NEAR_PERSON):
                 Q_des, command_result = self.controller_manager.head_priority_eye_trajectory_look_at_point()

                 self.process_task_result(Q_des, False)                 

                 if (command_result == True):
                    self.behavior_commanded = False
                    self.task_commanded = False

            elif (self.current_task == TASK_FOLLOW_WAYPOINTS):
                 if time != None:
                    time -= self.task_params[self.current_task_index-1][2] 
                 Q_des, command_result = self.controller_manager.head_eye_trajectory_follow(time)
                 self.process_task_result(Q_des, command_result)                 

            return

        #--------------
        # STATE GO HOME
        elif (self.current_state == STATE_GO_HOME):

            # BLOCK All Behavior Commands
            self.current_behavior = BEHAVIOR_NO_BEHAVIOR 
            if (self.gui_command_executing == False):
                self.gui_command_executing = True
                self.send_go_home_low_level_command()
                # send go home low level program command here

            if self.gui_command_executing:
                # Wait for a few seconds before changing state back to IDLE
                time_interval = self.ROS_current_time - self.gui_command_execute_time
                if (time_interval > self.wait_time_go_home):
                    # Reset all Joints to Home Position
                    self.reset_all_joints_to_home()

                    self.gui_command_executing = False
                    # Change State Back To Idle
                    self.current_state = STATE_IDLE
        #--------------
        else:
            print "ERROR Not a valid state" 


    # --------------------------------------------------------
    # Main Program Loop Logic
    # 1. Update time variables
    # --Loop--
    # 2. Set up behavior and update behavior variables (Needs to run once per button click)
    # 3. Update variables that will command a single task sequentially from behavior list (Needs to run until the behavior is completed)
    # 4. Compute the joint positions needed for the specified state from the task variables updated previously
    # 5. Send the joint positions to the controller (Rviz/dreamer head)
    # 6. Publish focus length?
    # 7. Do people related things?
    # 8. Print debugging information
    # 9. Update loop time
    def loop(self):

        while not rospy.is_shutdown():
            self.update_time()

            self.interval = self.ROS_current_time - self.time_since_last_loop                       

            if self.interval > (1.0/( float(NODE_RATE))):
                # Process Logic
                self.behavior_logic()
                self.task_logic()
                self.state_logic()
                # Send Commands
                self.send_command()
                # Visualization
                self.gaze_focus_states.publish_focus_length()
                
                # Find People
                # self.people_manager.loop()

                # Print state information
                # self.print_debug()
                # Sleep
                self.time_since_last_loop = self.ROS_current_time
                # print '        Node Rate (Hz)      :' , 1.0/self.interval #self.dt 

            if self.publish_to_low_level:
                self.publish_low_level_commands()
            if self.calculate_low_level:
                self.calculate_behavior_joints(20.0)




if __name__ == "__main__":
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()