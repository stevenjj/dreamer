#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import util_quat as quat
import head_kinematics as hk

global head_kin
head_kin = hk.Head_Kinematics()

MAX_EYE_VEL = 0.5 # Maximum Eye Velocity


def calculate_dQ(J, dx):
    dQ = np.linalg.pinv(J).dot(dx)
    return dQ

def circular_trajectory(t):
    radius = 0.4
    x_offset, y_offset, z_offset = 1.0, 0, 0.0
    T = 6.0
    frequency = 1.0/T
    angular_frequency = 2*3.14*frequency
    x = x_offset
    y = y_offset + radius*np.cos(angular_frequency*t)
    z = z_offset + radius*np.sin(angular_frequency*t)   

    return np.array([x, y, z])
    # Publish x(t) markers.

# Calculate desired orientation
# Accepts a 3x1 gaze location vector and an origin vector to create the desired orientation

def calc_smooth_desired_orientation(x_gaze_loc, p_cur, Q_cur, Q_init, scaling, orientation_type='head'):
    # Calculate Desired Orientation
    p_bar = x_gaze_loc - p_cur
    x_hat_d = p_bar/np.linalg.norm(p_bar)

    if (orientation_type == 'head'):
        R_init, p_init = head_kin.get_6D_Head_Position(Q_init)
    elif (orientation_type == 'right_eye'):
        R_init, p_init = head_kin.get_6D_Right_Eye_Position(Q_init)
    elif (orientation_type == 'left_eye'):        
        R_init, p_init = head_kin.get_6D_Left_Eye_Position(Q_init)
    else:
        raise 'unknown position and orientation needed'


    # if mr.NearZero(np.linalg.norm(p_bar)):
    #     print "HELLO?"

    z_world_hat = np.array([0,0,1])
    y_world_hat = np.array([0,1,0])    
    # Threshold before we use y_world_hat
    epsilon = 0.1 * np.pi/180.0 # degrees in radians 
    phi = np.arccos( (x_hat_d.dot(z_world_hat)) / (np.linalg.norm(x_hat_d)*np.linalg.norm(z_world_hat)) )

    z_hat_o = z_world_hat
    if (phi <= epsilon):
        z_hat_o = y_world_hat

#    x_hat_cur = np.array(R_init)[:,0]
    z_hat_init = np.array(R_init)[:,2]
#    print 'preffered z difference:', z_hat_o - z_hat_init   
    z_hat_o = mr.Normalize(z_hat_init + (z_hat_o-z_hat_init)*scaling)
    #z_hat_o = z_hat_cur


    z_hat_d = mr.Normalize(z_hat_o - (z_hat_o.dot(x_hat_d)*x_hat_d))
    y_hat_d = np.cross(z_hat_d, x_hat_d)


    R_desired = np.array([x_hat_d, y_hat_d, z_hat_d]).T

    # For some desired final roll configuration head/eye roll:
    # perform body frame twist about x^ direction. z_hat_d and y_hat_d will rotate as appropriate
    #   let phi be the total roll in radians. set theta = phi*s(t) with s(t) being the minimum jerk scaling
    #    R_desired = R_desired*Rot(world_x_hat, theta) 

    # print 'Desired Orientation '
    # print 'x_hat', x_hat_d.T
    # print 'y_hat', y_hat_d.T
    # print 'z_hat', z_hat_d.T
#    print R_desired

    return R_desired




# Calculate error between current configuration and desired configuration
def smooth_orientation_error(x_gaze_loc, Q, Q_init, scaling, orientation_type='head'):
    global head_kin
    J = None
    R_cur, p_cur = None, None
    # Get forward kinematics
    if (orientation_type == 'head'):
        R_cur, p_cur = head_kin.get_6D_Head_Position(Q)
    elif (orientation_type == 'right_eye'):
        R_cur, p_cur = head_kin.get_6D_Right_Eye_Position(Q)
    elif (orientation_type == 'left_eye'):        
        R_cur, p_cur = head_kin.get_6D_Left_Eye_Position(Q)
    else:
        raise 'unknown position and orientation needed'

    # Calculate desired orientation
    R_des = calc_smooth_desired_orientation(x_gaze_loc, p_cur, Q, Q_init, scaling, orientation_type)

    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))

    return quat.quat_to_wth(q_error)

def rotation_quaternion_error(R_cur, R_des):
    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))

    theta = 2*np.arccos(q_error[0])

    #print theta, q_error[0], q_error
    if (mr.NearZero(1.0 - q_error[0])):
        return 0, q_error[1:]

    factor = np.sin(theta/2.0)
    w_hat_x = q_error[1]/factor
    w_hat_y = q_error[2]/factor
    w_hat_z = q_error[3]/factor        

    angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])

    return theta, angular_vel_hat    


def calc_desired_orientation(x_gaze_loc, p_cur, Q_cur, orientation_type='head'):
    # Calculate Desired Orientation
    p_bar = x_gaze_loc - p_cur
    x_hat_d = p_bar/np.linalg.norm(p_bar)

    if (orientation_type == 'head'):
        R_cur, p_cur = head_kin.get_6D_Head_Position(Q_cur)
    elif (orientation_type == 'right_eye'):
        R_cur, p_cur = head_kin.get_6D_Right_Eye_Position(Q_cur)
    elif (orientation_type == 'left_eye'):        
        R_cur, p_cur = head_kin.get_6D_Left_Eye_Position(Q_cur)
    else:
        raise 'unknown position and orientation needed'

    z_world_hat = np.array([0,0,1])
    y_world_hat = np.array([0,1,0])    
    # Threshold before we use y_world_hat
    epsilon = 0.1 * np.pi/180.0 # degrees in radians 
    phi = np.arccos( (x_hat_d.dot(z_world_hat)) / (np.linalg.norm(x_hat_d)*np.linalg.norm(z_world_hat)) )

    z_hat_o = z_world_hat
    if (phi <= epsilon):
        z_hat_o = y_world_hat

    z_hat_d = mr.Normalize(z_hat_o - (z_hat_o.dot(x_hat_d)*x_hat_d))
    y_hat_d = np.cross(z_hat_d, x_hat_d)

    R_desired = np.array([x_hat_d, y_hat_d, z_hat_d]).T
    # print 'Desired Orientation '
    #print 'x_hat', x_hat_d.T
    #print 'y_hat', y_hat_d.T
    #print 'z_hat', z_hat_d.T
#    print R_desired

    return R_desired



# Calculate error between current configuration and desired configuration
def orientation_error(x_gaze_loc, Q, orientation_type='head'):
    global head_kin
    J = None
    R_cur, p_cur = None, None
    # Get forward kinematics
    if (orientation_type == 'head'):
        R_cur, p_cur = head_kin.get_6D_Head_Position(Q)
    elif (orientation_type == 'right_eye'):
        R_cur, p_cur = head_kin.get_6D_Right_Eye_Position(Q)
    elif (orientation_type == 'left_eye'):        
        R_cur, p_cur = head_kin.get_6D_Left_Eye_Position(Q)
    else:
        raise 'unknown position and orientation needed'

    # Calculate desired orientation
    R_des = calc_desired_orientation(x_gaze_loc, p_cur, Q)

    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))

    return quat.quat_to_wth(q_error)

def rotation_quaternion_error(R_cur, R_des):
    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))

    theta = 2*np.arccos(q_error[0])

    #print theta, q_error[0], q_error
    if (mr.NearZero(1.0 - q_error[0])):
        return 0, q_error[1:]

    factor = np.sin(theta/2.0)
    w_hat_x = q_error[1]/factor
    w_hat_y = q_error[2]/factor
    w_hat_z = q_error[3]/factor        

    angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])

    return theta, angular_vel_hat



class Controller():
    H = "head"
    RE = "right_eye"
    LE = "left_eye"

    def __init__(self, head_kinematics, gaze_focus_states, people_manager):
        self.kinematics = head_kinematics 
        self.gaze_focus_states = gaze_focus_states
        self.people_manager = people_manager        

        self.start_time = 0
        self.current_traj_time = 0
        self.prev_traj_time = 0
        self.movement_duration = 1
        self.epsilon = 0.01

        self.xyz_gaze_loc = np.array([0,0,0])

        self.Q_o_at_start = self.kinematics.Jlist

        self.trajectory_length = {self.H : 1, self.RE : 1, self.LE : 1 }

        self.gaze_focus_states.current_focus_length = {self.H : 1, self.RE : 1, self.LE : 1 }


        self.xyz_head_gaze_loc = np.array([0,0,0])
        self.xyz_eye_gaze_loc = np.array([0,0,0])


    def specify_head_eye_gaze_point(self, start_time, xyz_head_gaze_loc, xyz_eye_gaze_loc, movement_duration):
        # Initialize kinematic positions
        self.Q_o_at_start = self.kinematics.Jlist

        # Initialize Final Head Gaze Location Point
        self.xyz_head_gaze_loc = xyz_head_gaze_loc
        self.xyz_eye_gaze_loc = xyz_eye_gaze_loc

        # Initialize Time parameters
        self.start_time = start_time
        self.current_traj_time = start_time
        self.prev_traj_time = 0     


        self.initialize_head_eye_focus_point(xyz_head_gaze_loc, xyz_eye_gaze_loc)

        # Set Minimum Jerk parameters
        # Set total cartesian trajectory length
        self.trajectory_length[self.H]  = np.linalg.norm(xyz_head_gaze_loc - self.gaze_focus_states.focus_point_init[self.H] )
        self.trajectory_length[self.RE] = np.linalg.norm(xyz_eye_gaze_loc - self.gaze_focus_states.focus_point_init[self.RE] )        
        self.trajectory_length[self.LE] = np.linalg.norm(xyz_eye_gaze_loc - self.gaze_focus_states.focus_point_init[self.LE] )        

        # Set total DT to move
        self.movement_duration = movement_duration

        #raise 'debug'
        return


    def initialize_head_eye_focus_point(self, xyz_head_gaze_loc, xyz_eye_gaze_loc):
        R_head_init, p_head_init = self.kinematics.get_6D_Head_Position(self.kinematics.Jlist)
        R_right_eye_init, p_right_eye_init = self.kinematics.get_6D_Right_Eye_Position(self.kinematics.Jlist)
        R_left_eye_init, p_left_eye_init = self.kinematics.get_6D_Left_Eye_Position(self.kinematics.Jlist)                

        x_head_hat = np.array(R_head_init)[:,0]
        x_right_eye_hat = np.array(R_right_eye_init)[:,0]        
        x_left_eye_hat = np.array(R_left_eye_init)[:,0]

        # If focused, use that point as the initial_gaze_point
        if self.gaze_focus_states.are_eyes_focused():
            self.gaze_focus_states.focus_point_init[self.H]  = (p_head_init      + x_head_hat*self.gaze_focus_states.current_focus_length[self.H])
            self.gaze_focus_states.focus_point_init[self.RE] = (p_right_eye_init + x_right_eye_hat*self.gaze_focus_states.current_focus_length[self.RE])        
            self.gaze_focus_states.focus_point_init[self.LE] = (p_left_eye_init  + x_left_eye_hat*self.gaze_focus_states.current_focus_length[self.LE])
        else:
            self.gaze_focus_states.focus_length[self.H]  =  np.linalg.norm(xyz_head_gaze_loc - p_head_init)
            self.gaze_focus_states.focus_length[self.RE] = np.linalg.norm(xyz_eye_gaze_loc - p_right_eye_init)        
            self.gaze_focus_states.focus_length[self.LE] = np.linalg.norm(xyz_eye_gaze_loc - p_left_eye_init)

            self.gaze_focus_states.focus_point_init[self.H]  = (p_head_init      + x_head_hat*self.gaze_focus_states.focus_length[self.H])
            self.gaze_focus_states.focus_point_init[self.RE] = (p_right_eye_init + x_right_eye_hat*self.gaze_focus_states.focus_length[self.RE])        
            self.gaze_focus_states.focus_point_init[self.LE] = (p_left_eye_init  + x_left_eye_hat*self.gaze_focus_states.focus_length[self.LE])

    def xi_to_xf_vec(self, t, initial_point, final_point):
        x_f, x_i = final_point, initial_point
        e = x_f - x_i
        L = np.linalg.norm(x_f - x_i)
        e_hat = mr.Normalize(e)

        if mr.NearZero(L):
            return mr.Normalize(x_i), L
        return e_hat, L


    def min_jerk_time_scaling(self, t, delta_t): #delta_t is the total movement duration
        s_t = 0.0
        x_init, x_final = 0.0, 1.0 # Set to 0 and 1 since we want time scaling from 0 to 1
        if (t < 0):
            s_t = 0.0
        elif (t >= delta_t):
            s_t = 1.0
        else:
            s_t = x_init + (x_final-x_init)*( 10.0*((t/delta_t)**3.0) - 15.0*((t/delta_t)**4.0) + 6.0*((t/delta_t)**5.0) )
        return s_t 


    def calculate_t_t_prev(self):
        self.current_traj_time = rospy.Time.now().to_sec() - self.start_time
        t = self.current_traj_time
        t_prev = self.prev_traj_time    
        return t, t_prev



    # Fixed Head, Move Eyes Task Only
    def head_priority_eye_trajectory_look_at_point(self):
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = t - t_prev
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config and jacobian
        xyz_eye_gaze_loc = self.xyz_eye_gaze_loc
        xyz_head_gaze_loc = self.xyz_head_gaze_loc


        # Specify current (x,y,z) gaze location, joint config and jacobian
        Q_cur = self.kinematics.Jlist
        J_head = self.kinematics.get_6D_Head_Jacobian(Q_cur)

        J1_bar = np.linalg.pinv(J_head)        
        pJ1_J1 = J1_bar.dot(J_head)

        I_1 = np.eye(np.shape(pJ1_J1)[0])
        N1 = I_1 - pJ1_J1
        #print N1

        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point
        x_i_head = self.gaze_focus_states.focus_point_init[self.H]

        #print '         Focus Point', x_i

        # Find vector from initial focus point to final focus point
        e_hat_head, L_head = self.xi_to_xf_vec(t, x_i_head, xyz_head_gaze_loc)
        # Current desired gaze point
        p_head_des_cur = x_i_head + e_hat_head*L_head*(self.min_jerk_time_scaling(t, DT))

        # Calculate current orientation error
        #d_theta_error, angular_vel_hat = smooth_orientation_error(p_head_des_cur, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT))
        d_theta_error, angular_vel_hat = smooth_orientation_error(p_head_des_cur, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT)) 
        dx1 = d_theta_error * angular_vel_hat
        dx1 = np.concatenate( (dx1, np.array([0,0,0])),  axis=1)
        dq1 = calculate_dQ(J_head, dx1)


        Q_des = Q_cur + dq1 

        Q_cur = self.kinematics.Jlist
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)

        #J_1 = J_1[0:3,:] #Grab the first 3 rows      
        #J_2 = J_2[0:3,:] #Grab the first 3 rows            

        J2 = np.concatenate((J_1,J_2) ,axis=0)
 

        pinv_J2_N1 = np.linalg.pinv(J2.dot(N1))
        J2_pinv_J1 = J2.dot(J1_bar)
        J2_pinv_J1_x1dot = (J2.dot(J1_bar)).dot(dx1)


        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point for each eye
        x_i_right_eye = self.gaze_focus_states.focus_point_init[self.RE]
        x_i_left_eye = self.gaze_focus_states.focus_point_init[self.LE]

        # Find vector from initial focus point to final focus point
        e_hat_re, L_re = self.xi_to_xf_vec(t, x_i_right_eye, xyz_eye_gaze_loc)
        e_hat_le, L_le = self.xi_to_xf_vec(t, x_i_left_eye, xyz_eye_gaze_loc)        

        # Current desired gaze point
        p_des_cur_re = x_i_right_eye + e_hat_re*L_re*(self.min_jerk_time_scaling(t, DT))
        p_des_cur_le = x_i_left_eye + e_hat_le*L_le*(self.min_jerk_time_scaling(t, DT))        

        # Calculate current orientation error for each eye
        d_theta_error_re, angular_vel_hat_re = smooth_orientation_error(p_des_cur_re, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT), 'right_eye') 
        d_theta_error_le, angular_vel_hat_le = smooth_orientation_error(p_des_cur_le, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT), 'left_eye')         
        dx_re = d_theta_error_re * angular_vel_hat_re
        dx_le = d_theta_error_le * angular_vel_hat_le

        dx_re = np.concatenate( (dx_re, np.array([0,0,0])),  axis=1)
        dx_le = np.concatenate( (dx_le, np.array([0,0,0])),  axis=1)

 
        dx = np.concatenate( (dx_re, dx_le),  axis=1)
 
        dq = N1.dot(pinv_J2_N1.dot(dx -J2_pinv_J1_x1dot))

        dq = np.dot(N1, dq)
        #dq = calculate_dQ(J2, dx)

        #Q_des = Q_cur + dq 

        Q_des = Q_des + dq

        #print '         d_theta_error_re', d_theta_error_re
        #print '         d_theta_error_le', d_theta_error_le


        self.prev_traj_time = t
        # Loop Done

        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'left_eye')

        #print 'Right Eye Th Error', theta_error_right_eye, 'rads ', (theta_error_right_eye*180.0/np.pi), 'degrees'      
        #print 'Left Eye Th Error', theta_error_left_eye, 'rads ', (theta_error_left_eye*180.0/np.pi), 'degrees'      


        R_cur_head, p_cur_head = self.kinematics.get_6D_Head_Position(Q_cur)
        R_cur, p_cur_right_eye = self.kinematics.get_6D_Right_Eye_Position(Q_cur)
        R_cur, p_cur_left_eye = self.kinematics.get_6D_Left_Eye_Position(Q_cur)


        self.gaze_focus_states.current_focus_length[self.H] =   np.linalg.norm(p_cur_head - p_head_des_cur)  
        self.gaze_focus_states.current_focus_length[self.RE] =  np.linalg.norm(p_cur_right_eye - p_des_cur_re)         
        self.gaze_focus_states.current_focus_length[self.LE] =  np.linalg.norm(p_cur_left_eye -p_des_cur_le)
 


        # Prepare result of command
        result = False
        if (t > DT):
            result = True

        return Q_des, result

    # Eye has higher priority than head
    def eye_priority_head_trajectory_look_at_point(self):
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = t - t_prev
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config and jacobian
        xyz_eye_gaze_loc = self.xyz_eye_gaze_loc
        xyz_head_gaze_loc = self.xyz_head_gaze_loc

        Q_cur = self.kinematics.Jlist
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)

        J_1 = J_1[0:3,:] #Grab the first 3 rows      
        J_2 = J_2[0:3,:] #Grab the first 3 rows  

        J_head = self.kinematics.get_6D_Head_Jacobian(Q_cur)
        J_head = J_head[0:3,:]        
    
        J1 = np.concatenate((J_1,J_2) ,axis=0)

        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point for each eye
        x_i_right_eye = self.gaze_focus_states.focus_point_init[self.RE]
        x_i_left_eye = self.gaze_focus_states.focus_point_init[self.LE]

        # Find vector from initial focus point to final focus point
        e_hat_re, L_re = self.xi_to_xf_vec(t, x_i_right_eye, xyz_eye_gaze_loc)
        e_hat_le, L_le = self.xi_to_xf_vec(t, x_i_left_eye, xyz_eye_gaze_loc)        

        # Current desired gaze point
        p_des_cur_re = x_i_right_eye + e_hat_re*L_re*(self.min_jerk_time_scaling(t, DT))
        p_des_cur_le = x_i_left_eye + e_hat_le*L_le*(self.min_jerk_time_scaling(t, DT))        

        # Calculate current orientation error for each eye
        d_theta_error_re, angular_vel_hat_re = smooth_orientation_error(p_des_cur_re, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT), 'right_eye') 
        d_theta_error_le, angular_vel_hat_le = smooth_orientation_error(p_des_cur_le, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT), 'left_eye')         
        dx_re = d_theta_error_re * angular_vel_hat_re
        dx_le = d_theta_error_le * angular_vel_hat_le

        #dx_re = np.concatenate( (dx_re, np.array([0,0,0])),  axis=1)
        #dx_le = np.concatenate( (dx_le, np.array([0,0,0])),  axis=1)

 
        dx1 = np.concatenate( (dx_re, dx_le),  axis=1)
 
 
        dq1 = calculate_dQ(J1, dx1)

        Q_des = Q_cur + dq1 

        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'left_eye')

        #print 'Right Eye Th Error', theta_error_right_eye, 'rads ', (theta_error_right_eye*180.0/np.pi), 'degrees'      
        #print 'Left Eye Th Error', theta_error_left_eye, 'rads ', (theta_error_left_eye*180.0/np.pi), 'degrees'      


        J1_bar = np.linalg.pinv(J1)        
        pJ1_J1 = J1_bar.dot(J1)

        # print np.shape(np.linalg.pinv(J1.T))
        # print np.shape(np.linalg.pinv(J1.T).dot(J1.T))
        # print np.shape(J1_bar), np.shape(pJ1_J1)

        I_1 = np.eye(np.shape(pJ1_J1)[0])
        N1 = I_1 - pJ1_J1

        J2 = J_head
        pinv_J2_N1 = np.linalg.pinv(J2.dot(N1))
        J2_pinv_J1 = J2.dot(J1_bar)
        J2_pinv_J1_x1dot = (J2.dot(J1_bar)).dot(dx1)

        # Head task second

        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point
        x_i_head = self.gaze_focus_states.focus_point_init[self.H]

        #print '         Focus Point', x_i

        # Find vector from initial focus point to final focus point
        e_hat_head, L_head = self.xi_to_xf_vec(t, x_i_head, xyz_head_gaze_loc)
        # Current desired gaze point
        p_head_des_cur = x_i_head + e_hat_head*L_head*(self.min_jerk_time_scaling(t, DT))

        # Calculate current orientation error
        #d_theta_error, angular_vel_hat = smooth_orientation_error(p_head_des_cur, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT))
        d_theta_error, angular_vel_hat = smooth_orientation_error(p_head_des_cur, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT)) 
        dx2 = d_theta_error * angular_vel_hat
        #dx2 = np.concatenate( (dx2, np.array([0,0,0])),  axis=1)
        
        

        #dq2 = N1.dot(pinv_J2_N1.dot(dx2 -J2_pinv_J1_x1dot))

        dq2 = calculate_dQ(J_head, dx2)
        dq2 = np.dot(N1, dq2)
        

        #Q_des = Q_cur + dq2
        Q_des = Q_des + dq2


        J2_bar = np.linalg.pinv(J2)        
        pJ2_J2 = J2_bar.dot(J2)

        # print np.shape(np.linalg.pinv(J1.T))
        # print np.shape(np.linalg.pinv(J1.T).dot(J1.T))
        # print np.shape(J1_bar), np.shape(pJ1_J1)

        I_2 = np.eye(np.shape(pJ2_J2)[0])
        N2 = I_2 - pJ2_J2

        #print N1.dot(N2)
        #print dt

        #Q_des = Q_des + N1.dot(N2.dot(-0.01*(Q_des-Q_cur)/dt))
        #Q_des = Q_des + N1.dot(N2.dot(1.0*(Q_cur-Q_des)))        

        self.prev_traj_time = t
        # Loop Done

        R_cur_head, p_cur_head = self.kinematics.get_6D_Head_Position(Q_cur)
        R_cur, p_cur_right_eye = self.kinematics.get_6D_Right_Eye_Position(Q_cur)
        R_cur, p_cur_left_eye = self.kinematics.get_6D_Left_Eye_Position(Q_cur)


        self.gaze_focus_states.current_focus_length[self.H] =   np.linalg.norm(p_cur_head - p_head_des_cur)  
        self.gaze_focus_states.current_focus_length[self.RE] =  np.linalg.norm(p_cur_right_eye - p_des_cur_re)         
        self.gaze_focus_states.current_focus_length[self.LE] =  np.linalg.norm(p_cur_left_eye -p_des_cur_le)
 

        # Prepare result of command
        result = False
        if (t > DT):
            result = True

        return Q_des, result

    def control_track_person(self, dt):
        human_eye_pos = np.array([1, 0, self.kinematics.l1])
        if (len(self.people_manager.list_of_people) > 0):
            human_eye_pos = self.people_manager.identify_closest_person_from_gaze_focus()            


        tracking_condition = self.track_person_condition(human_eye_pos)

        if not(tracking_condition):
            human_eye_pos = np.array([1,0,0])

        Q_res = self.kinematics.Jlist
        result = False
        if not( ( np.linalg.norm(human_eye_pos - self.gaze_focus_states.focus_point_init[self.RE]) < 0.05 ) and \
                ( np.linalg.norm(human_eye_pos - self.gaze_focus_states.focus_point_init[self.LE]) < 0.05 ) ):

            # Call initialized
            start_time = rospy.get_time()
            xyz_eye_gaze_loc = human_eye_pos
            xyz_head_gaze_loc = human_eye_pos
            movement_duration = 1
            # movement_duration = 0 
            self.specify_head_eye_gaze_point(start_time, xyz_head_gaze_loc, xyz_eye_gaze_loc, movement_duration)
            # Decide which control law to use

            # Feed des_velocity to controller
            Q_des, result = self.velocity_track_eye_priority_head_look_at_point(dt, MAX_EYE_VEL)
            #Q_des, result = self.velocity_track_head_priority_eye_look_at_point(dt, MAX_EYE_VEL)            
            Q_res = Q_des

        return Q_res, result

    def track_person_condition(self, human_pos):
        (x,y,z) = human_pos[0], human_pos[1], human_pos[2] 
        if ( ((z < -0.2) or (z > 0.5)) and ((x < 0.2) or (x > 3)) ):
            return False

        x_hat = np.array([1,0,0])
        dp = (human_pos.dot(np.array([1,0,0])))
        x_hat_norm = np.linalg.norm( x_hat )
        xyz_person_norm = np.linalg.norm( human_pos )

        phi = np.arccos(x_hat.dot(human_pos) / xyz_person_norm / x_hat_norm)

        if ((dp > 0) and (phi < 1.0472)): #(np.pi/4.0))):
            return True
        else:
            return False


    def sat(self, x):
        if np.abs(x) <= 1:
            return x
        else:
            return np.sign(x) 

    # Eye has higher priority than head
    def velocity_track_eye_priority_head_look_at_point(self, dt_in, DES_VEL):
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = dt_in
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config and jacobian
        xyz_eye_gaze_loc = self.xyz_eye_gaze_loc
        xyz_head_gaze_loc = self.xyz_head_gaze_loc

        Q_cur = self.kinematics.Jlist
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)

        J_1 = J_1[0:3,:] #Grab the first 3 rows      
        J_2 = J_2[0:3,:] #Grab the first 3 rows  

        J_head = self.kinematics.get_6D_Head_Jacobian(Q_cur)
        J_head = J_head[0:3,:]        
    
        J1 = np.concatenate((J_1,J_2) ,axis=0)

        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point for each eye
        x_i_right_eye = self.gaze_focus_states.focus_point_init[self.RE]
        x_i_left_eye = self.gaze_focus_states.focus_point_init[self.LE]

        # Find vector from initial focus point to final focus point
        e_hat_re, L_re = self.xi_to_xf_vec(t, x_i_right_eye, xyz_eye_gaze_loc)
        e_hat_le, L_le = self.xi_to_xf_vec(t, x_i_left_eye, xyz_eye_gaze_loc)        

        # v_des_re = e_hat_re * DES_VEL 
        # v_des_le = e_hat_le * DES_VEL        

        # v_re_command = v_des_re #v_cur_re - v_des_re 
        # v_le_command = v_des_le #v_cur_le - v_des_le       

        # -------------------
        re_pos_error = (xyz_eye_gaze_loc - x_i_right_eye)
        kv = 0.5
        v_re_des = kv*re_pos_error
        v_re_sat = self.sat(MAX_EYE_VEL/np.linalg.norm(v_re_des))

        le_pos_error = (xyz_eye_gaze_loc - x_i_left_eye)
        v_le_des = kv*le_pos_error
        v_le_sat = self.sat(MAX_EYE_VEL/np.linalg.norm(v_le_des))

        v_re_command = v_re_sat*v_re_des
        v_le_command = v_le_sat*v_le_des        
        #------------------------

        # Current desired gaze point
        p_des_cur_re = x_i_right_eye +  v_re_command*dt #e_hat_le*0.01 #np.array([0.1,0.1,0])  #v_des_re*dt #np.array([0,0,0.1]) #v_re_command*dt#  e_hat_re*dt # np.array([5,0,0])#+ v_re_command*dt #e_hat_re*L_re*(self.min_jerk_time_scaling(t, DT))
        p_des_cur_le = x_i_left_eye  +  v_le_command*dt #e_hat_le*0.01 #np.array([0.1,0.1,0]) #v_des_le*dt #np.array([0,0,0.1]) #v_le_command*dt#e_hat_le*dt #+ v_le_command*dt  #e_hat_le*L_le*(self.min_jerk_time_scaling(t, DT))    


        # p_des_cur_re = x_i_cur + v_sat*v_des * dt
        # v_sat = sat(V_max / norm(v_des))
        # sat(x) = x        if |x| <= 1.0
        #        = sgn(x)   if |x| > 1.0
        # scaling = v_sat*v_des * dt


        # Calculate current orientation error for each eye
        d_theta_error_re, angular_vel_hat_re = orientation_error(p_des_cur_re, Q_cur, 'right_eye') #smooth_orientation_error(p_des_cur_re, Q_cur, self.Q_o_at_start, 0.1, 'right_eye') 
        d_theta_error_le, angular_vel_hat_le = orientation_error(p_des_cur_le, Q_cur, 'left_eye') #smooth_orientation_error(p_des_cur_le, Q_cur, self.Q_o_at_start, 0.1, 'left_eye')         

        dx_re = d_theta_error_re * angular_vel_hat_re
        dx_le = d_theta_error_le * angular_vel_hat_le


        dx1 = np.concatenate( (dx_re, dx_le),  axis=1)
        dq1 = calculate_dQ(J1, dx1)

        Q_des = Q_cur + dq1 

        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'left_eye')

        J1_bar = np.linalg.pinv(J1)        
        pJ1_J1 = J1_bar.dot(J1)

        I_1 = np.eye(np.shape(pJ1_J1)[0])
        N1 = I_1 - pJ1_J1

        J2 = J_head
        pinv_J2_N1 = np.linalg.pinv(J2.dot(N1))
        J2_pinv_J1 = J2.dot(J1_bar)
        J2_pinv_J1_x1dot = (J2.dot(J1_bar)).dot(dx1)


        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point

        x_i_head = self.gaze_focus_states.focus_point_init[self.H]

        #print '         Focus Point', x_i

        # Find vector from initial focus point to final focus point
        e_hat_head, L_head = self.xi_to_xf_vec(t, x_i_head, xyz_head_gaze_loc)


#        v_des_h = e_hat_head * DES_VEL 


        h_pos_error = (xyz_head_gaze_loc - x_i_head)
        kv = 2.0
        v_h_des = kv*h_pos_error
        v_h_sat = self.sat(MAX_EYE_VEL/np.linalg.norm(v_re_des))

        v_des_h = v_h_sat*v_h_des



        # Current desired gaze point
        p_head_des_cur = x_i_head + v_des_h*dt 





        # Calculate current orientation error
        #d_theta_error, angular_vel_hat = smooth_orientation_error(p_head_des_cur, Q_cur, self.Q_o_at_start, self.min_jerk_time_scaling(t,DT))
        d_theta_error, angular_vel_hat = orientation_error(p_head_des_cur, Q_cur, 'head')
        dx2 = d_theta_error * angular_vel_hat

        dq2 = calculate_dQ(J_head, dx2)
        dq2 = np.dot(N1, dq2)     
        Q_des = Q_des + dq2


        J2_bar = np.linalg.pinv(J2)        
        pJ2_J2 = J2_bar.dot(J2)

        I_2 = np.eye(np.shape(pJ2_J2)[0])
        N2 = I_2 - pJ2_J2

        # self.prev_traj_time = t
        # # Loop Done

        R_cur_head, p_cur_head = self.kinematics.get_6D_Head_Position(Q_cur)
        R_cur, p_cur_right_eye = self.kinematics.get_6D_Right_Eye_Position(Q_cur)
        R_cur, p_cur_left_eye = self.kinematics.get_6D_Left_Eye_Position(Q_cur)


        self.gaze_focus_states.current_focus_length[self.H] =   np.linalg.norm(p_cur_head - xyz_eye_gaze_loc)  
        self.gaze_focus_states.current_focus_length[self.RE] =  np.linalg.norm(p_cur_right_eye - xyz_eye_gaze_loc)         
        self.gaze_focus_states.current_focus_length[self.LE] =  np.linalg.norm(p_cur_left_eye -xyz_eye_gaze_loc)
 

        # Prepare result of command
        result = False

        return Q_des, result




   # Fixed Head, Move Eyes Task Only
    def velocity_track_head_priority_eye_look_at_point(self, dt_in, DES_VEL):
        # Calculate Time
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = dt_in
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config and jacobian
        xyz_eye_gaze_loc = self.xyz_eye_gaze_loc
        xyz_head_gaze_loc = self.xyz_head_gaze_loc


        # Specify current (x,y,z) gaze location, joint config and jacobian
        Q_cur = self.kinematics.Jlist
        J_head = self.kinematics.get_6D_Head_Jacobian(Q_cur)

        J1_bar = np.linalg.pinv(J_head)        
        pJ1_J1 = J1_bar.dot(J_head)

        I_1 = np.eye(np.shape(pJ1_J1)[0])
        N1 = I_1 - pJ1_J1
        #print N1

        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point
        x_i_head = self.gaze_focus_states.focus_point_init[self.H]

        #print '         Focus Point', x_i

        # Find vector from initial focus point to final focus point
        e_hat_head, L_head = self.xi_to_xf_vec(t, x_i_head, xyz_head_gaze_loc)

        v_des_h = e_hat_head * DES_VEL 

        # Current desired gaze point
        p_head_des_cur = x_i_head + v_des_h*dt 

        # Calculate current orientation error
        d_theta_error, angular_vel_hat = orientation_error(p_head_des_cur, Q_cur, 'head')
        dx1 = d_theta_error * angular_vel_hat
        dx1 = np.concatenate( (dx1, np.array([0,0,0])),  axis=1)
        dq1 = calculate_dQ(J_head, dx1)


        Q_des = Q_cur + dq1 

        Q_cur = self.kinematics.Jlist
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)

        #J_1 = J_1[0:3,:] #Grab the first 3 rows      
        #J_2 = J_2[0:3,:] #Grab the first 3 rows            

        J2 = np.concatenate((J_1,J_2) ,axis=0)
 

        pinv_J2_N1 = np.linalg.pinv(J2.dot(N1))
        J2_pinv_J1 = J2.dot(J1_bar)
        J2_pinv_J1_x1dot = (J2.dot(J1_bar)).dot(dx1)


        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point for each eye
        x_i_right_eye = self.gaze_focus_states.focus_point_init[self.RE]
        x_i_left_eye = self.gaze_focus_states.focus_point_init[self.LE]


        # Find vector from initial focus point to final focus point
        e_hat_re, L_re = self.xi_to_xf_vec(t, x_i_right_eye, xyz_eye_gaze_loc)
        e_hat_le, L_le = self.xi_to_xf_vec(t, x_i_left_eye, xyz_eye_gaze_loc)        

        v_des_re = e_hat_re * DES_VEL 
        v_des_le = e_hat_le * DES_VEL        

        v_re_command = v_des_re #v_cur_re - v_des_re 
        v_le_command = v_des_le #v_cur_le - v_des_le       


        # Current desired gaze point
        p_des_cur_re = x_i_right_eye +  v_re_command*dt #e_hat_le*0.01 #np.array([0.1,0.1,0])  #v_des_re*dt #np.array([0,0,0.1]) #v_re_command*dt#  e_hat_re*dt # np.array([5,0,0])#+ v_re_command*dt #e_hat_re*L_re*(self.min_jerk_time_scaling(t, DT))
        p_des_cur_le = x_i_left_eye  +  v_le_command*dt #e_hat_le*0.01 #np.array([0.1,0.1,0]) #v_des_le*dt #np.array([0,0,0.1]) #v_le_command*dt#e_hat_le*dt #+ v_le_command*dt  #e_hat_le*L_le*(self.min_jerk_time_scaling(t, DT))    


        # p_des_cur_re = x_i_cur + v_sat*v_des * dt
        # v_sat = sat(V_max / norm(v_des))
        # sat(x) = x        if |x| <= 1.0
        #        = sgn(x)   if |x| > 1.0
        # scaling = v_sat*v_des * dt


        # Calculate current orientation error for each eye
        d_theta_error_re, angular_vel_hat_re = orientation_error(p_des_cur_re, Q_cur, 'right_eye') #smooth_orientation_error(p_des_cur_re, Q_cur, self.Q_o_at_start, 0.1, 'right_eye') 
        d_theta_error_le, angular_vel_hat_le = orientation_error(p_des_cur_le, Q_cur, 'left_eye') #smooth_orientation_error(p_des_cur_le, Q_cur, self.Q_o_at_start, 0.1, 'left_eye')         

        dx_re = d_theta_error_re * angular_vel_hat_re
        dx_le = d_theta_error_le * angular_vel_hat_le


        dx_re = np.concatenate( (dx_re, np.array([0,0,0])),  axis=1)
        dx_le = np.concatenate( (dx_le, np.array([0,0,0])),  axis=1)

 
        dx = np.concatenate( (dx_re, dx_le),  axis=1)
 
        dq = N1.dot(pinv_J2_N1.dot(dx -J2_pinv_J1_x1dot))

        dq = np.dot(N1, dq)
        #dq = calculate_dQ(J2, dx)

        #Q_des = Q_cur + dq 

        Q_des = Q_des + dq

        #print '         d_theta_error_re', d_theta_error_re
        #print '         d_theta_error_le', d_theta_error_le


        self.prev_traj_time = t
        # Loop Done

        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_eye_gaze_loc, Q_cur, 'left_eye')

        #print 'Right Eye Th Error', theta_error_right_eye, 'rads ', (theta_error_right_eye*180.0/np.pi), 'degrees'      
        #print 'Left Eye Th Error', theta_error_left_eye, 'rads ', (theta_error_left_eye*180.0/np.pi), 'degrees'      


        R_cur_head, p_cur_head = self.kinematics.get_6D_Head_Position(Q_cur)
        R_cur, p_cur_right_eye = self.kinematics.get_6D_Right_Eye_Position(Q_cur)
        R_cur, p_cur_left_eye = self.kinematics.get_6D_Left_Eye_Position(Q_cur)


        self.gaze_focus_states.current_focus_length[self.H] =   np.linalg.norm(p_cur_head - p_head_des_cur)  
        self.gaze_focus_states.current_focus_length[self.RE] =  np.linalg.norm(p_cur_right_eye - p_des_cur_re)         
        self.gaze_focus_states.current_focus_length[self.LE] =  np.linalg.norm(p_cur_left_eye -p_des_cur_le)
 


        # Prepare result of command
        result = False
        if (t > DT):
            result = True

        return Q_des, result