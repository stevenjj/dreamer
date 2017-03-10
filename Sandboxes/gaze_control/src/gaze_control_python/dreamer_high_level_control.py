#!/usr/bin/env python
import rospy

# Math Dependencies
import numpy as np
import modern_robotics as mr
import head_kinematics as hk
import util_quat as quat

from traj_manager import *
from detected_people_manager import *

# ROS 
import dreamer_joint_publisher
from gaze_control.srv import RunProgram, RunProgramRequest
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
import tf

from GUI_params import *

# Program Constants
JOINT_LIM_BOUND = 0.9 #between 0 to 1.0
NODE_RATE = 1000 # Update rate of this node in Hz
CMD_RATE = 50 # Update rate for publishing joint positions to client



#-----------------------------------------------
# ROS Client for Low Level control 
def setup_ctrl_deq_append():
    rospy.wait_for_service('ctrl_deq_append')
    ctrl_deq_append = None
    try:
        ctrl_deq_append = rospy.ServiceProxy('ctrl_deq_append', HeadJointCmd)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None
    else:
        return ctrl_deq_append

def setup_run_program():
    rospy.wait_for_service('run_gaze_program')
    run_gaze_program = None
    try:
        run_gaze_program = rospy.ServiceProxy('run_gaze_program', RunProgram)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return None
    else:
        return run_gaze_program
#-----------------------------------------------

# State List
IDLE = 0
GO_TO_POINT = 1


class Dreamer_Head():
    def __init__(self):
        self.behaviors =[]
        self.states = [IDLE, GO_TO_POINT]
        self.tasks = []

        # Setup Robot Kinematics
        self.kinematics = hk.Head_Kinematics() 

        self.joint_publisher = dreamer_joint_publisher.Custom_Joint_Publisher()
        self.GUI_listener    = rospy.Subscriber(GUI_CMD_TOPIC, Int8, self.gui_callback)

        self.LOW_LEVEL_CONTROL = False

        if self.LOW_LEVEL_CONTROL:
            self.ctrl_deq_append = setup_ctrl_deq_append()
            self.run_gaze_program = setup_run_program()
        else:
            self.ctrl_deq_append, self.run_gaze_program = False, False

        # Node rates
        # ROS Rate
        self.rate = rospy.Rate(NODE_RATE) 
        self.ROS_start_time = rospy.get_time()
        self.ROS_current_time = self.ROS_start_time        
        self.relative_time = self.ROS_current_time - self.ROS_start_time

        # Command Rate
        self.CMD_rate = CMD_RATE # rate to send commands to low levelin Hz
        self.time_since_last_cmd_sent = self.ROS_start_time
        self.cmd_rate_measured = 0 #self.CMD_rate

        # GUI Commands
        self.gui_command = NO_COMMAND
        self.gui_command_string = NO_COMMAND_STRING

    def loop(self):
        while not rospy.is_shutdown():
            self.update_time()
            self.send_low_level_commands()
            self.print_debug()
            self.rate.sleep()

    def update_head_joints(self, head_joint_list):
        self.kinematics.Jlist = head_joint_list

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


    def update_time(self):
        self.ROS_current_time = rospy.get_time()
        self.relative_time =  self.ROS_current_time - self.ROS_start_time                 
        return

    def prepare_joint_command(self):
        # Only control the first 4 joints for now
        joints = range(4)
        joint_rads = self.kinematics.Jlist[0:4]
        return joints, joint_rads

    def send_low_level_commands(self):      
        cmd_interval = self.ROS_current_time - self.time_since_last_cmd_sent
        if (cmd_interval > (1.0/( float(self.CMD_rate))) ):

            if self.LOW_LEVEL_CONTROL:
                print 'sending low level commands now'
                # Send latest joint list
                # client.req(self.kinematics.Jlist)
            self.cmd_rate_measured = 1.0/cmd_interval
            self.time_since_last_cmd_sent = rospy.get_time()
            self.joint_publisher.publish_joints()
        return

    def print_debug(self):
        print 'ROS time (sec): ', self.relative_time
        print '    LL Command Rate (Hz)', self.cmd_rate_measured  
        print '    GUI Command', self.gui_command_string


    def gui_callback(self, msg):
        gui_command = msg.data
        if gui_command == LOW_LEVEL_OFF:
            print gui_command, LOW_LEVEL_OFF_STRING
            self.gui_command = gui_command
            self.gui_command_string = LOW_LEVEL_OFF_STRING

        elif gui_command == LOW_LEVEL_ON:
            print gui_command, LOW_LEVEL_ON_STRING            
            self.gui_command = gui_command
            self.gui_command_string = LOW_LEVEL_ON_STRING

        elif gui_command == STATE_TO_IDLE:
            print gui_command, STATE_TO_IDLE_STRING
            self.gui_command = gui_command
            self.gui_command_string = STATE_TO_IDLE_STRING
        
        elif gui_command == GO_HOME:
            print gui_command, GO_HOME_STRING
            self.gui_command = gui_command
            self.gui_command_string = GO_HOME_STRING
        
        elif gui_command == DO_SQUARE_FIXED_EYES:  
            print gui_command, DO_SQUARE_FIXED_EYES_STRING
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_FIXED_EYES_STRING

        elif gui_command == DO_SQUARE_FIXED_HEAD:
            print gui_command, DO_SQUARE_FIXED_HEAD_STRING
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_FIXED_HEAD_STRING
        
        elif gui_command == DO_SQUARE_EYE_PRIORITY:
            print gui_command, DO_SQUARE_EYE_PRIORITY_STRING
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_EYE_PRIORITY_STRING
        
        elif gui_command == DO_SQUARE_HEAD_PRIORITY:
            print gui_command, DO_SQUARE_HEAD_PRIORITY_STRING
            self.gui_command = gui_command
            self.gui_command_string = DO_SQUARE_HEAD_PRIORITY_STRING
        
        elif gui_command == TRACK_NEAR_PERSON:
            print gui_command, TRACK_NEAR_PERSON_STRING
            self.gui_command = gui_command
            self.gui_command_string = TRACK_NEAR_PERSON_STRING
        
        elif gui_command == AVOID_NEAR_PERSON:
            print gui_command, AVOID_NEAR_PERSON_STRING
            self.gui_command = gui_command
            self.gui_command_string = AVOID_NEAR_PERSON_STRING
        
        elif gui_command == DO_WAYPOINT_TRAJ:      
            print gui_command, DO_WAYPOINT_TRAJ_STRING      
            self.gui_command = gui_command
            self.gui_command_string = DO_WAYPOINT_TRAJ_STRING
        
        else:
            print gui_command, "Invalid Command"




if __name__ == "__main__":
    rospy.init_node('dreamer_high_level_control')
    dreamer_head = Dreamer_Head()
    dreamer_head.loop()