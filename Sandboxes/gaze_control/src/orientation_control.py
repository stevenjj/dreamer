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

	def __init__(self):
		self.screw_axes_tables = np.array([self.S0, self.S1, self.S2, self.S3, self.S4, self.S5, self.S6])
		self.Jlist = np.zeros( np.shape(self.screw_axes_tables)[0] )						

	# Returns a 6xn Jacobian
	def get_6D_Head_Jacobian(self, JList):
		Slist = self.screw_axes_tables[0:4].T # Only first four joints affect head orientation
		thetalist = JList[0:4] # Only first four joints affect Head Orientation		
		return mr.JacobianSpace(Slist, thetalist)

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

	def get_command(self):
		return 0

if __name__ == '__main__':
	kin = Head_Kinematics()
	print kin.S0
#        rospy.init_node('head_joint_publisher')
#        custom_joint_publisher = Custom_Joint_Publisher() 
#        custom_joint_publisher.loop()
#    except rospy.ROSInterruptException:
#        pass
