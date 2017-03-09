#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np


# Assumes that all trajectories start at t = 0 and end at Delta_T
class MinJerk():
	def __init__(self, max_vel = 0.5): #max_vel is in m/s
		self.coeffs = np.zeros((3,6)) #coeffs for x,y,z axes
		self.max_vel = max_vel
		self.Delta_Ts = np.array([1,1,1])

	# x_final is a 6x1 vector
	# axis = x,y,z = 0,1,2
	def get_min_jerk_coeffs(self, to, tf, x_init, x_final, axis=0):
		T_matrix_coeffs = np.array([ 
			[1, to, to**2,   to**3,       to**4,      to**5 ],
			[0,  1,  2*to, 3*(to**2), 4*(to**3),   5*(to**6)],
			[0,  0,     2,    6*(to), 12*(to**2), 20*(to**3)],
			[1, tf, tf**2,   tf**3,       tf**4,      tf**5 ],
			[0,  1,  2*tf, 3*(tf**2), 4*(tf**3),   5*(tf**6)],
			[0,  0,     2,    6*(tf), 12*(tf**2), 20*(tf**3)],
			])
		boundary_conds = np.array([x_init[0], x_init[1], x_init[2], x_final[0], x_final[1], x_final[2]])
		coeffs = np.linalg.inv(T_matrix_coeffs).dot(boundary_conds)

		self.coeffs[axis] = coeffs
		return coeffs

	# xyz_init = np.array([ [x, xdot, xddot], 
	#  					  [y, ydot, yddot],
	# 					  [z, zdot, zddot] ])
	def get_all_min_jerk_coeffs(self, xyz_init, xyz_final, DT_des=1):
		to = 0
		for i in range(3):
			print xyz_final[i][0]
			xf, xi = xyz_final[i][0], xyz_init[i][0]
			DT = DT_des
			if not(mr.NearZero(xyz_final[i][1])):
				DT = (xf - xi) / self.bound_this_vel(xyz_final[i][1])
			self.Delta_Ts[i] = DT
			self.coeffs[i] = self.get_min_jerk_coeffs(to, to+DT, xyz_init[i], xyz_final[i], i)	


	def bound_this_vel(self, des_vel):
		desired_vel = des_vel
		if (des_vel > self.max_vel):
			desired_vel = self.max_vel
		return desired_vel

	# returns s_t t > 0
	def s_t(self, time, axis=0):
		t = time
		if (t < 0):
			t = 0
		elif (t > self.Delta_Ts[axis]):
			t = self.Delta_Ts[axis]
		else:
			t = time

		t_vec = np.array([1, t, t**2,   t**3,       t**4,      t**5 ])
		return t_vec.dot(self.coeffs[axis])



'''
sample_min_jerk = MinJerk()
# xyz_init = np.array([ [xi, xidot, xiddot], 
#  					  [yi, yidot, yiddot],
# 					  [zi, zidot, ziddot] ])
# xyz_final = np.array([ [xf, xfdot, xfddot], 
#  					  [yf, yfdot, yfddot],
# 					  [zf, zfdot, zfddot] ])

xyz_init = np.array([[0, 0, 0], 
  					  [0, 0, 0],
 					  [0, 0, 0] ])

des_x_vel, des_y_vel, des_z_vel = 0.9, 0.9, 0

des_vel = np.array([des_x_vel, des_y_vel, des_z_vel])
xyz_final = np.array([  [1, des_x_vel, 0], 
  					    [1, des_y_vel, 0],
 					    [1, des_z_vel, 0] ])

sample_min_jerk.get_all_min_jerk_coeffs(xyz_init, xyz_final)
print sample_min_jerk.coeffs[0]
print sample_min_jerk.coeffs[1]
print sample_min_jerk.coeffs[2]
print sample_min_jerk.s_t(3, 0)
print sample_min_jerk.s_t(3, 1)
print sample_min_jerk.s_t(3, 2)
print sample_min_jerk.Delta_Ts
'''