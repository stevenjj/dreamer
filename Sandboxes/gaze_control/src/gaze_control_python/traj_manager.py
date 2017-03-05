#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np
import util_quat as quat
import head_kinematics as hk

global head_kin
head_kin = hk.Head_Kinematics()

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
def calc_desired_orientation(x_gaze_loc, p_cur):
    # Calculate Desired Orientation
    z_world_hat = np.array([0,0,1])
    y_world_hat = np.array([0,1,0])    

    p_bar = x_gaze_loc - p_cur
    x_hat_d = p_bar/np.linalg.norm(p_bar)

    # Threshold before we use y_world_hat
    epsilon = 0.1 * np.pi/180.0 # degrees in radians 
    phi = np.arccos( (x_hat_d.dot(z_world_hat)) / (np.linalg.norm(x_hat_d)*np.linalg.norm(z_world_hat)) )

    z_hat_o = z_world_hat
    if (phi <= epsilon):
        z_hat_o = y_world_hat

    z_hat_d = mr.Normalize(z_hat_o - (z_hat_o.dot(x_hat_d)*x_hat_d))
    y_hat_d = np.cross(z_hat_d, x_hat_d)

    R_desired = np.array([x_hat_d, y_hat_d, z_hat_d]).T
#    print 'Desired Orientation '
#    print 'x_hat', x_hat_d.T
#    print 'y_hat', y_hat_d.T
#    print 'z_hat', z_hat_d.T
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
    R_des = calc_desired_orientation(x_gaze_loc, p_cur)

    q_cur = quat.R_to_quat(R_cur)
    q_des = quat.R_to_quat(R_des)
    q_error = quat.quat_multiply(q_des, quat.conj(q_cur))

    return quat.quat_to_wth(q_error)
   #  theta = 2*np.arccos(q_error[0])

   #  #print theta, q_error[0], q_error
   #  if (mr.NearZero(1.0 - q_error[0])):
   #      return 0, q_error[1:]

   #  factor = np.sin(theta/2.0)
   #  w_hat_x = q_error[1]/factor
   #  w_hat_y = q_error[2]/factor
   #  w_hat_z = q_error[3]/factor        

   #  angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])

   # # print theta, angular_vel_hat
   #  return theta, angular_vel_hat

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


# Need an eye focus task:
#   1) set eyes to point towards head orientation
#   2) set eyes to point towards a point

class Trajectory_Manager():
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


    def specify_gaze_point(self, start_time, Q_cur, xyz_gaze_loc, movement_duration, eyes_focused = False, eye_focal_point = np.array([1,0,0])):
        # Initialize kinematic positions
        self.kinematics.Jlist = Q_cur # This should be updated first!
        self.Q_o_at_start = Q_cur

        # Initialize Final Gaze Location Point
        self.xyz_gaze_loc = xyz_gaze_loc

        # Initialize Time parameters
        self.start_time = start_time
        self.current_traj_time = start_time
        self.prev_traj_time = 0     

        self.initialize_eye_focus_point(xyz_gaze_loc, Q_cur, eyes_focused)

        # Set Minimum Jerk parameters
        # Set total cartesian trajectory length
        self.trajectory_length[self.H]  = np.linalg.norm(xyz_gaze_loc - self.focus_point_init[self.H] )
        self.trajectory_length[self.RE] = np.linalg.norm(xyz_gaze_loc - self.focus_point_init[self.RE] )        
        self.trajectory_length[self.LE] = np.linalg.norm(xyz_gaze_loc - self.focus_point_init[self.LE] )        

        # Set total DT to move
        self.movement_duration = movement_duration

        #raise 'debug'
        return

    def initialize_eye_focus_point(self, xyz_gaze_loc, Q_cur, eyes_focused = False, eye_focal_point = np.array([1,0,0])):
        R_head_init, p_head_init = self.kinematics.get_6D_Head_Position(Q_cur)
        R_right_eye_init, p_right_eye_init = self.kinematics.get_6D_Right_Eye_Position(Q_cur)
        R_left_eye_init, p_left_eye_init = self.kinematics.get_6D_Left_Eye_Position(Q_cur)                

        x_head_hat = np.array(R_head_init)[:,0]
        x_right_eye_hat = np.array(R_right_eye_init)[:,0]        
        x_left_eye_hat = np.array(R_left_eye_init)[:,0]

        print '             p_head_init', p_head_init
        print '             R_head_init', np.array(R_head_init)
        print '             x_head_hat', x_head_hat


        # If focused, use that point as the initial_gaze_point
        if (eyes_focused):
            # Do stuff ehre
            self.focus_point_init = self.focus_point_init
        else:       
            self.focus_length[self.H] = np.linalg.norm(xyz_gaze_loc - p_head_init)
            self.focus_length[self.RE] = np.linalg.norm(xyz_gaze_loc - p_right_eye_init)        
            self.focus_length[self.LE] = np.linalg.norm(xyz_gaze_loc - p_left_eye_init)

            self.focus_point_init[self.H]  = (p_head_init      + x_head_hat*self.focus_length[self.H])
            self.focus_point_init[self.RE] = (p_right_eye_init + x_right_eye_hat*self.focus_length[self.RE])        
            self.focus_point_init[self.LE] = (p_left_eye_init  + x_left_eye_hat*self.focus_length[self.LE])



    # returns the unit vector direction and length from an initial (xyz) point to a final (xyz) point
    # initial_point and final_point are expected to be in the same frame (typically global)
    def xi_to_xf_vec(self, t, initial_point, final_point):
        x_f, x_i = final_point, initial_point

        e = x_f - x_i
        L = np.linalg.norm(x_f - x_i)
        e_hat = mr.Normalize(e)

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


    # Head Task Only
    def head_trajectory_look_at_point(self):
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = t - t_prev
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config and jacobian
        xyz_gaze_loc = self.xyz_gaze_loc
        Q_cur = self.kinematics.Jlist
        J = self.kinematics.get_6D_Head_Jacobian(Q_cur)
        #J = J[0:3,:] #Grab the first 3 rows

        
        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point
        x_i = self.focus_point_init[self.H]

        print '         Focus Point', x_i

        # Find vector from initial focus point to final focus point
        e_hat, L = self.xi_to_xf_vec(t, x_i, xyz_gaze_loc)
        # Current desired gaze point
        p_des_cur = x_i + e_hat*L*(self.min_jerk_time_scaling(t, DT))

        # Calculate current orientation error
        d_theta_error, angular_vel_hat = orientation_error(p_des_cur, Q_cur)
        dx = d_theta_error * angular_vel_hat
        dx = np.concatenate( (dx, np.array([0,0,0])),  axis=1)
        dq = calculate_dQ(J, dx)
        Q_des = Q_cur + dq 

        print '         desired dq', dq
        self.prev_traj_time = t
        # Loop Done

        # calculate current orientation
        R_current, p_current = self.kinematics.get_6D_Head_Position(Q_cur)
        q_cur = quat.R_to_quat(R_current)

        # calculate current desired orientation
        R_des = calc_desired_orientation(xyz_gaze_loc, p_current)
        q_des = quat.R_to_quat(R_des)        

        theta_error, angular_vel_hat = orientation_error(xyz_gaze_loc, Q_cur)
        print '      Theta error', (theta_error*180.0/np.pi), 'degrees'      



        # Prepare result of command
        result = False
        if (t > DT):
            result = True

        return Q_des, result


    # Eye Task Only
    def eye_trajectory_look_at_point(self):
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = t - t_prev
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config and jacobian
        xyz_gaze_loc = self.xyz_gaze_loc
        Q_cur = self.kinematics.Jlist
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)

        #J_1 = J_1[0:3,:] #Grab the first 3 rows      
        #J_2 = J_2[0:3,:] #Grab the first 3 rows            

        J = np.concatenate((J_1,J_2) ,axis=0)
 

        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)

        # Calculate Current Desired Gaze Point
        # Get initial focus point for each eye
        x_i_right_eye = self.focus_point_init[self.RE]
        x_i_left_eye = self.focus_point_init[self.LE]

        print '         Focus Point right_eye', x_i_right_eye
        print '         Focus Point left_eye', x_i_left_eye


        # Find vector from initial focus point to final focus point
        e_hat_re, L_re = self.xi_to_xf_vec(t, x_i_right_eye, xyz_gaze_loc)
        e_hat_le, L_le = self.xi_to_xf_vec(t, x_i_left_eye, xyz_gaze_loc)        

        # Current desired gaze point
        p_des_cur_re = x_i_right_eye + e_hat_re*L_re*(self.min_jerk_time_scaling(t, DT))
        p_des_cur_le = x_i_left_eye + e_hat_le*L_le*(self.min_jerk_time_scaling(t, DT))        

        # Calculate current orientation error for each eye
        d_theta_error_re, angular_vel_hat_re = orientation_error(p_des_cur_re, Q_cur, 'right_eye')
        d_theta_error_le, angular_vel_hat_le = orientation_error(p_des_cur_le, Q_cur, 'left_eye')        
        dx_re = d_theta_error_re * angular_vel_hat_re
        dx_le = d_theta_error_le * angular_vel_hat_le

        dx_re = np.concatenate( (dx_re, np.array([0,0,0])),  axis=1)
        dx_le = np.concatenate( (dx_le, np.array([0,0,0])),  axis=1)

 
        dx = np.concatenate( (dx_re, dx_le),  axis=1)
        dq = calculate_dQ(J, dx)

        #dq = calculate_dQ(J_1, dx_re)
        Q_des = Q_cur + dq 

        print '         desired dq', dq

        self.prev_traj_time = t
        # Loop Done

        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_gaze_loc, Q_cur, 'left_eye')

        #print 'Right Eye Th Error', theta_error_right_eye, 'rads ', (theta_error_right_eye*180.0/np.pi), 'degrees'      
        #print 'Left Eye Th Error', theta_error_left_eye, 'rads ', (theta_error_left_eye*180.0/np.pi), 'degrees'      



        # Prepare result of command
        result = False
        if (t > DT):
            result = True

        return Q_des, result
