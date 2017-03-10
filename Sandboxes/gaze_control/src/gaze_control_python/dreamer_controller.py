#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import util_quat as quat
import head_kinematics as hk

global head_kin
head_kin = hk.Head_Kinematics()

class Controller():
    H = "head"
    RE = "right_eye"
    LE = "left_eye"

    def __init__(self, head_kinematics):
        self.kinematics = head_kinematics 

        self.start_time = 0
        self.current_traj_time = 0
        self.prev_traj_time = 0
        self.movement_duration = 1
        self.epsilon = 0.01

        self.xyz_gaze_loc = np.array([0,0,0])

        self.theta_total_head = 0
        self.angular_vel_hat_head = np.array([0,0,0])        

        self.theta_total_right_eye = 0
        self.angular_vel_hat_right_eye = np.array([0,0,0])         
               
        self.theta_total_left_eye = 0
        self.angular_vel_hat_left_eye = np.array([0,0,0])        

        self.Q_o_at_start = self.kinematics.Jlist

        self.focus_length = {self.H : 1, self.RE : 1, self.LE : 1 }
        self.focus_point_init = {self.H : 1, self.RE : 1, self.LE : 1 }
        self.trajectory_length = {self.H : 1, self.RE : 1, self.LE : 1 }

        self.current_focus_length = {self.H : 1, self.RE : 1, self.LE : 1 }


        self.xyz_head_gaze_loc = np.array([0,0,0])
        self.xyz_eye_gaze_loc = np.array([0,0,0])

