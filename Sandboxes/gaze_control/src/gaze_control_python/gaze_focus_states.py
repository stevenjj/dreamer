#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import util_quat as quat
import head_kinematics as hk

from std_msgs.msg import Float32MultiArray

FOCUS_THRESH = 0.2
class Gaze_Focus_States():
    H = "head"
    RE = "right_eye"
    LE = "left_eye"
    def __init__(self, head_kinematics):
        self.kinematics = head_kinematics

        # self.focus_length = {self.H : 1, self.RE : 1, self.LE : 1 }
        # self.focus_point_init = {self.H : 1, self.RE : 1, self.LE : 1 }
        self.current_focus_length = {self.H : 1, self.RE : 1, self.LE : 1 }

        # Current Focus Location
        self.position = {self.H : np.zeros(3), self.RE : np.zeros(3), self.LE : np.zeros(3) }

        # Focus Velocity        
        self.old_position = {self.H : np.zeros(3), self.RE : np.zeros(3), self.LE : np.zeros(3) }
        self.velocity = {self.H : np.zeros(3), self.RE : np.zeros(3), self.LE : np.zeros(3) }

        # Update the Position of the Focus Initially
        self.update_focus_location()
        self.focus_length_pub = rospy.Publisher('setArrLength', Float32MultiArray, queue_size=1)

    # Update gaze focus states 
    def update_gaze_focus_states(self, dt): 
        # Assume that current self.position was computed using old self.kinematics.Jlist
        self.old_position = self.position
        self.update_focus_location()

        self.velocity[self.H] =  (self.old_position[H]  - self.position[self.H] ) /dt
        self.velocity[self.RE] = (self.old_position[RE] - self.position[self.RE])/dt
        self.velocity[self.LE] = (self.old_position[LE] - self.position[self.LE])/dt                

    # Update current focus location using latest joint positions
    def update_focus_location(self):
        R_head, p_head           = self.kinematics.get_6D_Head_Position(self.kinematics.Jlist)
        R_right_eye, p_right_eye = self.kinematics.get_6D_Right_Eye_Position(self.kinematics.Jlist)
        R_left_eye, p_left_eye   = self.kinematics.get_6D_Left_Eye_Position(self.kinematics.Jlist)

        x_head_eye_hat = np.array(R_right_eye)[:,0]        
        x_right_eye_hat = np.array(R_right_eye)[:,0]        
        x_left_eye_hat = np.array(R_left_eye)[:,0]

        head_fl = self.current_focus_length[self.H]
        re_fl = self.current_focus_length[self.RE]
        le_fl = self.current_focus_length[self.LE] 

        self.position[self.H]  = x_head_eye_hat*head_fl + p_head
        self.position[self.RE] = x_right_eye_hat*re_fl  + p_right_eye        
        self.position[self.LE] = x_left_eye_hat*le_fl   + p_head


    def are_eyes_focused(self):
        if(np.linalg.norm(self.position[self.RE] - self.position[self.LE])) < FOCUS_THRESH:
            return True
        else:
            return False

    def publish_focus_length(self):
        msg = Float32MultiArray()
        #common_length = 0.6
        #focus_lengths = [common_length, common_length, common_length + self.kinematics.l2]
        #msg.data = focus_lengths
        msg.data.append(self.current_focus_length[self.LE])                
        msg.data.append(self.current_focus_length[self.RE])
        msg.data.append(self.current_focus_length[self.H])
        self.focus_length_pub.publish(msg)


    def loop(self):
        self.publish_focus_length()