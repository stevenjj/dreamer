#!/usr/bin/env python
import rospy
import time
import math
import modern_robotics as mr
import numpy as np

class Head_Kinematics():
	# Dreamer Links
	l0 = 0.3115
	l1 = 0.13849
	l2 = 0.12508
	l3 = 0.053
	# # Dreamer Screw Axes @ 0 Position
	# # S = [omega_hat, v]
	# S0 = [0, -1, 0, l0, 0, 0]
	# S1 = [0, 0, 1, 0, 0, 0]
	# S2 = [-1, 0, 0, 0, l0+l1, 0]
	# S3 = [0, -1, 0, l0+l1, 0, 0]
	# S4 = [0, -1, 0, l0+l1, 0, l2]
	# S5 = [0,  0, 1, -l3, -l2, 0]
	# S6 = [0,  0, 1, l3, -l2, 0]					

	# R_head_home = np.eye(3)
	# #p_head_home = [0, 0, l0+l1]
	# p_head_home = [l3, 0, l0+l1]

	# R_right_eye_home = np.eye(3)
	# p_right_eye_home = [l2, -l3, l0+l1]	

	# R_right_eye_home = np.eye(3)
	# p_left_eye_home = [l2, l3, l0+l1]		

	# Dreamer Head from J0
	# Dreamer Screw Axes @ 0 Position
	# S = [omega_hat, v]

	S0 = [0, -1, 0, 0, 0, 0]
	S1 = [0, 0, 1, 0, 0, 0]
	S2 = [-1, 0, 0, 0, -l1, 0]
	S3 = [0, -1, 0, l1, 0, 0]
	S4 = [0, -1, 0, l1, 0, -l2]
	S5 = [0,  0, 1, -l3, -l2, 0]
	S6 = [0,  0, 1, l3, -l2, 0]

	R_head_home = np.eye(3)
	p_head_home = [0, 0, l1]
#	p_head_home = [l2, 0, l1]

	R_right_eye_home = np.eye(3)
	p_right_eye_home = [l2, -l3, l1]	

	R_left_eye_home = np.eye(3)
	p_left_eye_home = [l2, l3, l1]		


	def __init__(self):
		self.screw_axes_tables = np.array([self.S0, self.S1, self.S2, self.S3, self.S4, self.S5, self.S6])
		self.J_num = np.shape(self.screw_axes_tables)[0]
		self.Jlist = np.zeros( np.shape(self.screw_axes_tables)[0] )
		self.Jindex_to_names = {0: "lower_neck_pitch", \
								1: "upper_neck_yaw", \
								2: "upper_neck_roll", \
								3: "upper_neck_pitch", \
								4: "eye_pitch", \
								5: "right_eye_yaw", \
								6: "left_eye_yaw"}								

	# Returns the head's 6xn Jacobian
	def get_6D_Head_Jacobian(self, JList):
		screw_axis_end = 3 # This is J3
		num_joints = screw_axis_end + 1 # should be 4
		Slist = self.screw_axes_tables[0:num_joints].T # Only first four joints affect head orientation
		thetalist = JList[0:num_joints] # Only first four joints affect Head Orientation		
		J_spatial =  mr.JacobianSpace(Slist, thetalist) # returns 6x4 Spatial Jacobian

		# Concatenate zeros
		zero_vec = np.zeros((6, self.J_num - num_joints )) # this should be 6x3
		J_spatial_6D_head = np.concatenate((J_spatial, zero_vec), axis=1) 

		return J_spatial_6D_head # returns 6x7 Spatial Jacobian

	def get_6D_Right_Eye_Jacobian(self, JList):		
		screw_axis_end = 5 # This is J5
		num_joints = screw_axis_end + 1 # should be 6
		Slist = self.screw_axes_tables[0:num_joints].T # Only first six joints affect right eye orientation and position
		thetalist = JList[0:num_joints] # Only first six joints affect right eye orientation and position		
		J_spatial =  mr.JacobianSpace(Slist, thetalist) # returns 6x6 Spatial Jacobian

		# Concatenate zeros
		zero_vec = np.zeros((6, self.J_num - num_joints )) # this should be 6x1
		J_spatial_6D_right_eye = np.concatenate((J_spatial, zero_vec), axis=1) 

		return J_spatial_6D_right_eye # returns 6x7 Spatial Jacobian

	def get_6D_Left_Eye_Jacobian(self, JList):
		screw_axis_end = 6 # This is J6
		num_joints = 6
		
		# Only six joints (excluding J5) affect left eye orientation and position
		# J0, J1, J2, J3, J4, J6
		Slist = np.concatenate( (self.screw_axes_tables[0:5], self.screw_axes_tables[6].reshape(1,6)), axis = 0).T
		thetalist = np.concatenate( (JList[0:5], np.array([JList[screw_axis_end]]) ), axis = 0 ) 		
		# Spatial Jacobian is [J0, J1, J2, J3, J4, J6]. we have to add back in a J5 zero vector later
		J_spatial =  mr.JacobianSpace(Slist, thetalist) # returns 6x6 Spatial Jacobian

		# Now we have to construct the following Jacobian:
		# J = [J0, J1, J2, J3, J4, 0, J6]

		# Jacobian due to joint 5 zero vector
		zero_vec = np.zeros((6, self.J_num - num_joints )) # this should be 6x1

		# Construct last column and first 5 columns
		J6_Jacobian = J_spatial[:,-1].reshape(6,1) # Last column of J_spatial, which is J6's jacobian
		J0_to_J4_Jacobian = J_spatial[:,0:5] # First five joints before left eye  

		# Add J5 zero column vector
		J0_to_J5_Jacobian = np.concatenate( (J0_to_J4_Jacobian, zero_vec), axis=1)
		J_spatial_6D_left_eye = np.concatenate((J0_to_J5_Jacobian, J6_Jacobian), axis=1) 

		return J_spatial_6D_left_eye # returns 6x7 Spatial Jacobian


	def get_6D_Right_Eye_Jacobian_yaw_pitch(self, JList):		
		screw_axis_end = 5 # This is J5
		num_joints = screw_axis_end + 1 # should be 6
		Slist = self.screw_axes_tables[4:num_joints].T # Only J4 and J5 affect right eye pitch and yaw
		thetalist = JList[4:num_joints] # Only J4 and J5 affect right eye pitch and yaw		
		J_spatial =  mr.JacobianSpace(Slist, thetalist) # returns 6x2 Spatial Jacobian

		# Concatenate zeros
		zero_vec_begin = np.zeros((6, 4)) # this should be 6x4		
		zero_vec = np.zeros((6, 1)) # this should be 6x1

		J_spatial_6D_right_eye = np.concatenate((zero_vec_begin, J_spatial), axis=1)  # 6x6
		J_spatial_6D_right_eye = np.concatenate((J_spatial_6D_right_eye, zero_vec), axis=1) #6x7

		return J_spatial_6D_right_eye # returns 6x7 Spatial Jacobian

	def get_6D_Left_Eye_Jacobian_yaw_pitch(self, JList):	
		# Only two joint affect left eye yaw pitch
		# J4, J6
		Slist = np.concatenate( (self.screw_axes_tables[4].reshape(1,6), self.screw_axes_tables[6].reshape(1,6)), axis = 0).T
		thetalist = np.array([ JList[4], JList[6] ]) 		
		# Spatial Jacobian is [J4, J6]. we have to add back in a J5 zero vector later
		J_spatial =  mr.JacobianSpace(Slist, thetalist) # returns 6x2 Spatial Jacobian

		# Now we have to construct the following Jacobian:
		# J = [0, 0, 0, 0, J4, 0, J6]

		# Jacobian due to joint 5 zero vector
		zero_vec_begin = np.zeros((6, 4)) # this should be 6x4				
		zero_vec = np.zeros((6, 1)) # this should be 6x1

		# Construct last column and first 5 columns
		J4_Jacobian = J_spatial[:,0].reshape(6,1) # Pitch Joint J4's Jacobian
		J6_Jacobian = J_spatial[:,1].reshape(6,1) # Left Eye Yaw Joint. J6's jacobian


		# Add J5 zero column vector
		J_spatial_6D_left_eye = np.concatenate( (zero_vec_begin, J4_Jacobian), axis=1)
		J_spatial_6D_left_eye = np.concatenate((J_spatial_6D_left_eye, zero_vec), axis=1) 
		J_spatial_6D_left_eye = np.concatenate((J_spatial_6D_left_eye, J6_Jacobian), axis=1) 

		return J_spatial_6D_left_eye # returns 6x7 Spatial Jacobian		


	# Returns the head's orientation R, and spatial position p from T \in SE(3)
	def get_6D_Head_Position(self, JList):
		screw_axis_end = 3 # This is J3
		num_joints = screw_axis_end + 1 # should be 4

		Slist = self.screw_axes_tables[0:num_joints].T # Only first four joints affect head orientation
		thetalist = JList[0:num_joints] # Only first four joints affect Head Orientation		
		M_head_home = mr.RpToTrans(self.R_head_home, self.p_head_home)

		T_head = mr.FKinSpace(M_head_home, Slist, thetalist)

		# Head Orientation and Position
		R_head, p_head = mr.TransToRp(T_head)
		return (R_head, p_head)

	# Rerturns the right eye's orientation R and spatial position p from T \in SE(3)
	def get_6D_Right_Eye_Position(self, JList):		
		screw_axis_end = 5 # This is J5
		num_joints = screw_axis_end + 1 # should be 6

		Slist = self.screw_axes_tables[0:num_joints].T # Only first six joints affect head orientation
		thetalist = JList[0:num_joints] # Only first six joints affect Head Orientation

		# Home Orientation and Position of Right Eye		
		M_right_eye_home = mr.RpToTrans(self.R_right_eye_home, self.p_right_eye_home)
		# Forward Kinematics of Right Eye
		T_right_eye = mr.FKinSpace(M_right_eye_home, Slist, thetalist)

		# Right Eye Orientation and Position	
		R_right_eye, p_right_eye = mr.TransToRp(T_right_eye)
		return (R_right_eye, p_right_eye)


	# Rerturns the left eye's orientation R and spatial position p from T \in SE(3)
	def get_6D_Left_Eye_Position(self, JList):		
		screw_axis_end = 6 # This is J6
		num_joints = 6
		
		# Only six joints (excluding J5) the affect left eye orientation and position
		# J0, J1, J2, J3, J4, J6
		Slist = np.concatenate( (self.screw_axes_tables[0:5], self.screw_axes_tables[6].reshape(1,6)), axis = 0).T
		thetalist = np.concatenate( (JList[0:5], np.array([JList[screw_axis_end]]) ), axis = 0 )

		# Home Orientation and Position of Right Eye		
		M_left_eye_home = mr.RpToTrans(self.R_left_eye_home, self.p_left_eye_home)
		# Forward Kinematics of Right Eye
		T_left_eye = mr.FKinSpace(M_left_eye_home, Slist, thetalist)

		# Right Eye Orientation and Position	
		R_left_eye, p_left_eye = mr.TransToRp(T_left_eye)
		return (R_left_eye, p_left_eye)
