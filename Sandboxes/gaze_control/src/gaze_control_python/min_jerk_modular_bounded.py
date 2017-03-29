#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np

'''
TODO:
	- Initialize empty array and then be able to append the first element
	- Bound the velocity/acceleration
'''

'''
Questions:
'''
class Waypoint():
	def __init__(self, x, dot_x, ddot_x, Dt):
		self.s = np.array([x, dot_x, ddot_x])
		self.Dt = Dt

class MinimumJerk():
	def __init__(self, waypoint_list, max_vel = 0.5, max_accel = 0.1):
		self.coeffs = np.zeros((3,6)) #coeffs for x,y,z axes
		self.max_vel = max_vel
		self.max_accel = max_accel
		self.waypoint_list = absolute_time(waypoint_list)
	

	def bound_this_vel(self, des_vel):
		desired_vel = des_vel
		if (des_vel > self.max_vel):
			desired_vel = self.max_vel
		return desired_vel

	# Will create a list of times and points from the waypoint differences
	def absolute_time(waypoint_list):


	# Low level calculator given two waypoints, will be called by a higher loop
	# Returns the minimum jerk coefficients for the two specified points
	# Waypoints will be 1D arrays with 3 elements
	def point_min_jerk_coeffs(self, waypoint_i, waypoint_f, axis=0):
		T_matrix_coeffs = np.array([ 
			[1, to, to**2,   to**3,       to**4,      to**5 ],
			[0,  1,  2*to, 3*(to**2), 4*(to**3),   5*(to**4)],
			[0,  0,     2,    6*(to), 12*(to**2), 20*(to**3)],
			[1, tf, tf**2,   tf**3,       tf**4,      tf**5 ],
			[0,  1,  2*tf, 3*(tf**2), 4*(tf**3),   5*(tf**4)],
			[0,  0,     2,    6*(tf), 12*(tf**2), 20*(tf**3)],
			])

		boundary_conds = np.array([waypoint_i[0], waypoint_i[1], waypoint_i[2], waypoint_f[0], waypoint_f[1], waypoint_f[2]])	
		coeffs = (np.linalg.inv(T_matrix_coeffs)).dot(boundary_conds)

		self.coeffs[axis] = coeffs
		return coeffs

	# Main Function, will go through our np array and calculate all coefficients
	def all_min_jerk_coeffs(self, DT_des=1):
		num_waypoints = self.waypoints.shape[0] # Gets how many waypoints we have

x_coord = []
x_coord.append(Waypoint(0, 0, 0, 1))
x_coord.append(Waypoint(10, 0, 0, 0))
print x_coord[1].s

test_move = MinimumJerk(x_coord)
test_move.point_min_jerk_coeffs(test_move.waypoint_list[0], test_move.waypoint_list[1])