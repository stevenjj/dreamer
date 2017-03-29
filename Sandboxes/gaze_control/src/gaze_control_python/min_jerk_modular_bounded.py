#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np

'''
TODO:
	- Initialize empty array and then be able to append the first element
	- Bound the velocity/acceleration
'''


class MinimumJerk():
	def __init__(self, max_vel = 0.5, max_accel = 0.1):
		self.coeffs = np.zeros((3,6)) #coeffs for x,y,z axes
		self.max_vel = max_vel
		self.max_accel = max_accel
		self.waypoints = self.read_Waypoints()
	
	def bound_this_vel(self, des_vel):
		desired_vel = des_vel
		if (des_vel > self.max_vel):
			desired_vel = self.max_vel
		return desired_vel

	# This function will read waypoints having 3 parameters
	# 	for now, it will be a hardcoded 2D array
	# Not a particularly useful function
	def read_Waypoints(self):
		#waypoint = np.empty([1,3])
		return np.array([[0,0,0]])

	# Add waypoint(s) onto the end of the array of waypoints
	# Input must be in the form of a 2D array
	def append_waypoint(self, waypoint):
		self.waypoints = np.concatenate((self.waypoints, waypoint), axis=0)

	# Low level calculator given two waypoints, will be called by a higher loop
	# Returns the minimum jerk coefficients for the two specified points
	# Waypoints will be 1D arrays with 3 elements
	def get_min_jerk_coeffs(self, to, tf, waypoint_i, waypoint_f, axis=0):
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
	


x_coord = MinimumJerk()
print "Initial Waypoints"
print x_coord.waypoints

x_coord.append_waypoint(np.array([[10,0,0]]))
print "Appended Waypoints"
print x_coord.waypoints

print x_coord.get_min_jerk_coeffs(0, .5, x_coord.waypoints[0], x_coord.waypoints[1])