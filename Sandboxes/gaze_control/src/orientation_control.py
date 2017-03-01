#!/usr/bin/env python
import rospy
import time
import math
import modern_robotics as mr
import numpy as np
import util_quat as quat
from head_kinematics import *
'''


'''
def calculate_dQ(J, dx):
	dQ = np.linalg.pinv(J).dot(dx)
	return dQ

def circular_trajectory(t):
    radius = 0.2
    x_offset, y_offset, z_offset = 0.25, 0, 0.01
    T = 6.0
    frequency = 1.0/T
    angular_frequency = 2*3.14*frequency
    x = x_offset
    y = y_offset + radius*np.cos(angular_frequency*t)
    z = z_offset + radius*np.sin(angular_frequency*t)	
    return np.array([x, y, z])
	# Publish x(t) markers.

# Calculate desired orientation
def calc_desired_orientation(x_gaze_loc, p_cur, orientation_type='head'):
    # Calculate Desired Orientation
    z_world_hat = np.array([0,0,1])
    y_world_hat = np.array([0,1,0])    

    p_bar = x_gaze_loc - p_cur
    x_hat_d = p_bar/np.linalg.norm(p_bar)

    # Threshold before we use y_world_hat
    epsilon = 3 * np.pi/180.0 # degrees in radians 
    phi = np.arccos( (x_hat_d.dot(z_world_hat)) / (np.linalg.norm(x_hat_d)*np.linalg.norm(z_world_hat)) )

    z_hat_o = z_world_hat
    if (phi <= epsilon):
        z_hat_o = y_world_hat

    z_hat_d = z_hat_o - z_hat_o.dot(x_hat_d)
    y_hat_d = np.cross(z_hat_d, x_hat_d)

    R_desired = np.array([x_hat_d.T, y_hat_d.T, z_hat_d.T]) 
    print 'Desired Orientation ', R_desired
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


    theta = 2*np.arccos(q_error[0])

    if (theta == 0):
        print theta, q_error[1:]
        return theta, q_error[1:]

    factor = np.sin(theta/2.0)
    w_hat_x = q_error[1]/factor
    w_hat_y = q_error[2]/factor
    w_hat_z = q_error[3]/factor        

    angular_vel_hat = np.array([w_hat_x, w_hat_y, w_hat_z])

    print theta, angular_vel_hat
    return theta, angular_vel_hat

# define zero position error
def zero_position_error():
    return np.array([0,0,0])


if __name__ == '__main__':
    #rospy.init_node('orientation_contro')
    global head_kin
    head_kin = Head_Kinematics() 
    kin = Head_Kinematics()
    print kin.S0
    
    orientation_error(np.array([1.053,0, 0.13849]) , kin.Jlist ,'head')

    J_head = kin.get_6D_Head_Jacobian(kin.Jlist)
    J_right_eye = kin.get_6D_Right_Eye_Jacobian(kin.Jlist)		
    J_left_eye = kin.get_6D_Left_Eye_Jacobian(kin.Jlist)
#        rospy.init_node('head_joint_publisher')
#        custom_joint_publisher = Custom_Joint_Publisher() 
#        custom_joint_publisher.loop()
#    except rospy.ROSInterruptException:
#        pass
