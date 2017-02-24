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
	l2 = 0.053
	l3 = 0.3115
	# Dreamer Screw Axes @ 0 Position
	# S = [omega_hat, v]
	S0 = [0, -1, 0, l0, 0, 0]
	S1 = [0, 0, 1, 0, 0, 0]
	S2 = [-1, 0, 0, 0, l0+l1, 0]
	S3 = [0, -1, 0, l0+l1, 0, 0]
	S4 = [0, -1, 0, l0+l1, 0, l2]
	S5 = [0,  0, 1, -l3, -l2, 0]
	S6 = [0,  0, 1, l3, -l2, 0]					

	R_head_home = np.eye(3)
	#p_head_home = [0, 0, l0+l1]
	p_head_home = [l3, 0, l0+l1]

	R_right_eye_home = np.eye(3)
	p_right_eye_home = [l2, -l3, l0+l1]	

	R_right_eye_home = np.eye(3)
	p_left_eye_home = [l2, l3, l0+l1]		

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
	def get_6D_Head_Jacobian(self, Jlist):
		screw_axis_end = 3 # This is J3
		num_joints = screw_axis_end + 1 # should be 4
		Slist = self.screw_axes_tables[0:num_joints].T # Only first four joints affect head orientation
		thetalist = Jlist[0:num_joints] # Only first four joints affect Head Orientation		
		J_spatial =  mr.JacobianSpace(Slist, thetalist) # returns 6x4 Spatial Jacobian

		# Concatenate zeros
		zero_vec = np.zeros((6, self.J_num - num_joints )) # this should be 6x3
		J_spatial_6D_head = np.concatenate((J_spatial, zero_vec), axis=1) 

		return J_spatial_6D_head

	# Returns The head's orientation R, and spatial position p from T \in SE(3)
	def get_6D_Head_Position(self, Jlist):
		screw_axis_end = 3 # This is J3
		num_joints = screw_axis_end + 1 # should be 4

		Slist = self.screw_axes_tables[0:num_joints].T # Only first four joints affect head orientation
		thetalist = Jlist[0:num_joints] # Only first four joints affect Head Orientation		
		M_head_home = mr.RpToTrans(self.R_head_home, self.p_head_home)

		T_head = mr.FKinSpace(M_head_home, Slist, thetalist)

		R_head, p_head = mr.TransToRp(T_head)
		return (R_head, p_head)


	def get_6D_Left_Eye_Jacobian(self, JList):
		return 0		
	def get_6D_Right_Eye_Jacobian(self, JList):		
		return 0


class Behavior_GUI():
	def get_command(self):
		return 0


class Orientation_Controller():
	def __init__(self):
		self.joints = {}
		self.maxVal = {}

	def get_command(self):
		# dx = [w1, w2, w3, x, y, z]	
		# np.linalg.pinv(J) * dx
		# dq = Jinv*dx
		return 0

	def get_error(self):

		return 0



if __name__ == '__main__':
	kin = Head_Kinematics()
	print kin.S0
	J = kin.get_6D_Head_Jacobian(kin.Jlist)
#        rospy.init_node('head_joint_publisher')
#        custom_joint_publisher = Custom_Joint_Publisher() 
#        custom_joint_publisher.loop()
#    except rospy.ROSInterruptException:
#        pass
