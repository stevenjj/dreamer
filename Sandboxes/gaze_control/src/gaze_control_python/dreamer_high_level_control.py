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
NODE_RATE = 10 # Update rate of this node in Hz
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
TASK_VEL_TRACK_EYE_PRIORITY       = 103
TASK_GO_TO_WAYPOINTS_EYE_PRIORITY = 104


TASK_ID_TO_STRING = {TASK_NO_TASK: "TASK_NO_TASK",
                     TASK_GO_TO_POINT_HEAD_PRIORITY: "TASK_GO_TO_POINT_HEAD_PRIORITY",
                     TASK_GO_TO_POINT_EYE_PRIORITY: "TASK_GO_TO_POINT_EYE_PRIORITY",
                     TASK_VEL_TRACK_EYE_PRIORITY: "TASK_VEL_TRACK_EYE_PRIORITY",
                     TASK_GO_TO_WAYPOINTS_EYE_PRIORITY: "TASK_GO_TO_WAYPOINTS_EYE_PRIORITY"}

# Behavior List
BEHAVIOR_NO_BEHAVIOR = 200
BEHAVIOR_DO_SQUARE_FIXED_EYES = 201
BEHAVIOR_DO_SQUARE_FIXED_HEAD = 202
BEHAVIOR_TRACK_NEAR_PERSON = 203
BEHAVIOR_ID_TO_STRING = {BEHAVIOR_NO_BEHAVIOR: "BEHAVIOR_NO_BEHAVIOR",
                         BEHAVIOR_DO_SQUARE_FIXED_EYES: "BEHAVIOR_DO_SQUARE_FIXED_EYES",
                         BEHAVIOR_DO_SQUARE_FIXED_HEAD: "BEHAVIOR_DO_SQUARE_FIXED_HEAD",
                         BEHAVIOR_TRACK_NEAR_PERSON: "BEHAVIOR_TRACK_NEAR_PERSON"
                    }

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
        self.controller_manager = Controller(self.kinematics, self.gaze_focus_states, self.people_manager)       

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



    #-------------------------------------------------------------
    # Joint and Gaze Focus Update
    def update_head_joints(self, head_joint_list):
        self.kinematics.Jlist = head_joint_list
        self.gaze_focus_states.update_gaze_focus_states(self.interval)

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
            self.joint_publisher.free_joints[joint_name]['position'] = joint_cmd_bound(command, joint_name, joint_max_val, joint_min_val)

    # Time Update
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
        hjc.numCtrlSteps.data = 550*self.interval #this should be CMD_RATE = 50 

        joints = joint_map
        rads = joint_val

        print rads

        hjc.joint_mapping.data = joints
        hjc.q_cmd_radians.data = rads

        # Send latest joint list
        self.ctrl_deq_append(hjc)
        return

    # -----------------------------------------------------------------
    # Send Joint Commands to RVIZ
    #   and to low Level if possible @ the control loop frequency
    def send_command(self):      
        cmd_interval = self.ROS_current_time - self.time_since_last_cmd_sent

        # if (cmd_interval > (1.0/( float(self.CMD_rate))) ):   
            # Send Low Level Commands if LL Control is enabled
        if (self.low_level_control and 
            self.current_state != STATE_IDLE and 
            self.current_state != STATE_GO_HOME):

            print 'sending low level commands now'
            self.send_low_level_commands()


        self.cmd_rate_measured = 1.0/self.interval #1.0/self.dt #1.0/cmd_interval
        self.time_since_last_cmd_sent = rospy.get_time()

        # Send to Rviz
        self.joint_publisher.publish_joints()
        return


    # ---------------------------------------------------------
    # Main State Machine
    #   State Machine computes current joint positions
    def state_logic(self):
        #--------------
        # STATE IDLE
        if (self.current_state == STATE_IDLE):
            return # Do Nothing

        #--------------
        # STATE GO TO POINT
        elif (self.current_state == STATE_GO_TO_POINT):
            if (self.current_task == TASK_GO_TO_POINT_EYE_PRIORITY):
                 Q_des, command_result = self.controller_manager.eye_priority_head_trajectory_look_at_point()
                 self.process_task_result(Q_des, command_result)

            elif (self.current_task == TASK_GO_TO_POINT_HEAD_PRIORITY):
                 Q_des, command_result = self.controller_manager.head_priority_eye_trajectory_look_at_point()                
                 self.process_task_result(Q_des, command_result)

            elif (self.current_task == TASK_VEL_TRACK_EYE_PRIORITY):
                 Q_des, command_result = self.controller_manager.control_track_person(self.interval)
                 self.process_task_result(Q_des, command_result)                 
            return

        #--------------
        # STATE GO HOME
        elif (self.current_state == STATE_GO_HOME):

            # BLOCK All Behavior Commands
            self.current_behavior = BEHAVIOR_NO_BEHAVIOR 
            if (self.gui_command_executing == False):
                # send go home low level program command here
                self.gui_command_executing = True

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
        

        elif gui_command == DO_SQUARE_EYE_PRIORITY:
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_EYE_PRIORITY_STRING
        
        elif gui_command == DO_SQUARE_HEAD_PRIORITY:
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_HEAD_PRIORITY_STRING
        
        elif gui_command == TRACK_NEAR_PERSON:
            self.gui_command = gui_command
            self.gui_command_string = TRACK_NEAR_PERSON_STRING
            self.reset_state_tasks_behaviors()
            self.current_behavior = BEHAVIOR_TRACK_NEAR_PERSON

        
        elif gui_command == AVOID_NEAR_PERSON:
            self.gui_command = gui_command
            self.gui_command_string = AVOID_NEAR_PERSON_STRING
        
        elif gui_command == DO_WAYPOINT_TRAJ:      
            self.gui_command = gui_command
            self.gui_command_string = DO_WAYPOINT_TRAJ_STRING
        
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

            # print self.current_task_index
            # print len(self.task_list)
            # print self.task_params

            #self.gaze_focus_states.print_debug()
            #self.gaze_focus_states.print_debug()
            #self.controller_manager.print_debug()
            self.people_manager.print_debug()


    # --------------------------------------------------------
    # Main Program Loop
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
                self.people_manager.loop()

                # Print out
                self.print_debug()
                # Sleep
                self.time_since_last_loop = self.ROS_current_time


    # Behavior and Task Logic


    #----------------------------------------------------------
    # Behavior Helper Functions
    def set_prioritized_go_to_point_params(self, head_gaze_location, eye_gaze_location, move_duration):
        params = (head_gaze_location, eye_gaze_location, move_duration)
        return params       

    def execute_behavior(self, task_list, task_params):
        # Update Task Variables
        self.task_list   = task_list
        self.task_params = task_params

        self.current_task_index = 0
        self.current_task = self.task_list[self.current_task_index]
        
        # Update Semaphores
        self.task_commanded     = False
        self.behavior_commanded = True


    def behavior_logic(self):
        if self.current_behavior == BEHAVIOR_NO_BEHAVIOR:
            self.task_list = [TASK_NO_TASK]
            self.task_params = []
            return

        # This behavior makes a square with the head while the eyes are pointing straight ahead
        elif ((self.current_behavior == BEHAVIOR_DO_SQUARE_FIXED_EYES) and self.behavior_commanded == False):
            task_list = [TASK_GO_TO_POINT_EYE_PRIORITY for i in range(6)]
            task_params = []

            init_to_go_point = 3 # Take a longer time to go to the initial point
            duration = 1
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1+0.3]),  np.array([6, 0.0, self.kinematics.l1]),      init_to_go_point) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1-0.3]),   np.array([6, 0.0, self.kinematics.l1]),     duration) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, -0.15, self.kinematics.l1-0.3]),  np.array([6, 0.0, self.kinematics.l1]),     duration) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, -0.15, self.kinematics.l1+0.3]), np.array([6, 0.0, self.kinematics.l1]),      duration) )                       
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1+0.3]),  np.array([6, 0.0, self.kinematics.l1]),      duration) )
            task_params.append( self.set_prioritized_go_to_point_params( np.array( [2, 0.15, self.kinematics.l1-0.3]),   np.array([6, 0.0, self.kinematics.l1]),     duration) )
            self.execute_behavior(task_list, task_params)

        elif ((self.current_behavior == BEHAVIOR_DO_SQUARE_FIXED_HEAD) and self.behavior_commanded == False):
            task_list = [TASK_GO_TO_POINT_HEAD_PRIORITY for i in range(6)]            
            task_params = []

            init_to_go_point = 3 # Take a longer time to go to the initial point
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      init_to_go_point) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, -0.15, self.kinematics.l1-0.2]),     1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, -0.15, self.kinematics.l1+0.2]),     1) )                       
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1+0.2]),      1) )
            task_params.append( self.set_prioritized_go_to_point_params(np.array( [1.0, 0.0, self.kinematics.l1]), np.array( [0.6, 0.15, self.kinematics.l1-0.2]),      1) )                   
            self.execute_behavior(task_list, task_params)


        elif ((self.current_behavior == BEHAVIOR_TRACK_NEAR_PERSON) and self.behavior_commanded == False):
            task_list = [TASK_VEL_TRACK_EYE_PRIORITY]            
            task_params = []
            self.execute_behavior(task_list, task_params)
        return

    def task_logic(self):
        if self.current_task == TASK_NO_TASK:
            return


        # Go to point task with the eyes having a higher priority
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

        elif ((self.current_task == TASK_VEL_TRACK_EYE_PRIORITY) and (self.task_commanded == False)):
            self.current_state = STATE_GO_TO_POINT
            self.task_commanded = True

        return


    def next_task(self):
        if (self.current_task_index < len(self.task_list)):
            self.current_task = self.task_list[self.current_task_index]

    def process_task_result(self, Q_des, command_result):
        self.update_head_joints(Q_des)
        if (command_result == True):
            self.task_commanded = False
            self.current_task_index += 1
            self.next_task()          

            if self.current_task_index == len(self.task_list):
                self.reset_state_tasks_behaviors()

    def reset_state_tasks_behaviors(self):
        self.current_state = STATE_IDLE
        self.current_task = TASK_NO_TASK 
        self.current_behavior = BEHAVIOR_NO_BEHAVIOR
        self.gui_command_executing = False
        self.behavior_commanded = False
        self.task_commanded = False


if __name__ == "__main__":
    rospy.init_node('dreamer_head_behavior')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()