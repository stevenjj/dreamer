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



class Trajectory_Manager():
    def __init__(self):
        self.kinematics = hk.Head_Kinematics() 

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


    # if error < xx return 'Change State'
    def specify_goal(self, start_time, Q_cur, xyz_gaze_loc, movement_duration):
        #print 'specify goal. current q:', Q_cur
        self.kinematics.Jlist = Q_cur
        self.xyz_gaze_loc = xyz_gaze_loc # 3x1 vector or np.array([x, y, z]) with shape 3,
        self.movement_duration = movement_duration
        self.start_time = start_time
        self.current_traj_time = start_time
        self.prev_traj_time = 0        

        self.theta_total, self.angular_vel_hat = orientation_error(xyz_gaze_loc, Q_cur)
        self.theta_total_right_eye, self.angular_vel_hat_right_eye = orientation_error(xyz_gaze_loc, Q_cur, 'right_eye')
        self.theta_total_left_eye, self.angular_vel_hat_left_eye = orientation_error(xyz_gaze_loc, Q_cur, 'left_eye')

        self.Q_o_at_start = Q_cur

        return

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

    def head_look_at_point(self):
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
        d_theta_error = self.theta_total*(self.min_jerk_time_scaling(t, DT) - self.min_jerk_time_scaling(t-dt, DT))
        dx = d_theta_error * self.angular_vel_hat
        dx = np.concatenate( (dx, np.array([0,0,0])),  axis=1)
        dq = calculate_dQ(J, dx)
        Q_des = Q_cur + dq 

        # Loop Done

        # Calculate Feedback ------------------------------------
        # calculate current Orientation
        R_current, p_current = self.kinematics.get_6D_Head_Position(Q_cur)
        q_cur = quat.R_to_quat(R_current)

        # calculate true desired orientation
        R_des = calc_desired_orientation(xyz_gaze_loc, p_current)
        q_des = quat.R_to_quat(R_des)        
        #q_feedback_error = quat.quat_multiply(q_des, quat.conj(q_cur))

        # Calculate feedback error
        theta_error, angular_vel_hat = orientation_error(xyz_gaze_loc, Q_cur)
        q_feedback_error = quat.wth_to_quat(angular_vel_hat, theta_error)

        # Add feedback term
        fe_dt_theta, fe_angular_vel = quat.quat_to_wth(q_feedback_error)
        dx_dw_fb = fe_angular_vel * fe_dt_theta
        dx_dx_fb =  np.array([0,0,0]) #np.array(p_init) - np.array(p_cur)
        dx_fb = np.concatenate( (dx_dw_fb, dx_dx_fb) ,  axis=1)

        dq_fb = calculate_dQ(J, dx_fb)
        #print 'current dq', dq, 'feedback dq', dq_fb
        Q_des = Q_des + 0.001*dq_fb

        self.kinematics.Jlist = Q_des
        self.prev_traj_time = t


        print ''
        print 'Theta Error', theta_error, 'rads ', (theta_error*180.0/np.pi), 'degrees'
        print '    q_cur =', q_cur
        print '    q_des =', q_des        
        print '    q fe  =', q_feedback_error

        # Prepare result of command
        result = False
        if (t > DT):
            result = True

        return Q_des, result

    # Eye Task Only
    def eyes_look_at_point(self):
        # Calculate Time
        t, t_prev = self.calculate_t_t_prev()
        dt = t - t_prev
        DT = self.movement_duration

        # Specify current (x,y,z) gaze location, joint config
        xyz_gaze_loc = self.xyz_gaze_loc
        Q_cur = self.kinematics.Jlist

        # Soecify Jacobian
        J_1 = self.kinematics.get_6D_Right_Eye_Jacobian(Q_cur)
        J_2 = self.kinematics.get_6D_Left_Eye_Jacobian(Q_cur)
        J_1 = J_1[0:3,:] #Grab the first 3 rows      
        J_2 = J_2[0:3,:] #Grab the first 3 rows            
        J = np.concatenate((J_1,J_2) ,axis=0)


        # Calculate FeedForward ---------------------------------
        # Calculate new Q_des (desired configuration)
        d_theta_error_right_eye = self.theta_total_right_eye*(self.min_jerk_time_scaling(t, DT) - self.min_jerk_time_scaling(t-dt, DT))
        d_theta_error_left_eye = self.theta_total_left_eye*(self.min_jerk_time_scaling(t, DT) - self.min_jerk_time_scaling(t-dt, DT))        
        dx_right_eye = d_theta_error_right_eye * self.angular_vel_hat_right_eye
        dx_left_eye = d_theta_error_left_eye * self.angular_vel_hat_left_eye        

        dx_two_tasks = np.concatenate((dx_right_eye, dx_left_eye), axis=1)
        dq = calculate_dQ(J, dx_two_tasks)
        Q_des = Q_cur + dq 

        theta_error_right_eye, angular_vel_hat_right_eye = orientation_error(xyz_gaze_loc, Q_cur, 'right_eye')
        theta_error_left_eye, angular_vel_hat_left_eye = orientation_error(xyz_gaze_loc, Q_cur, 'left_eye')
        print theta_error_right_eye, theta_error_left_eye, t

        result = False
        if (t > DT):
            result = True

        self.kinematics.Jlist = Q_des
        self.prev_traj_time = t

        return Q_des, result


