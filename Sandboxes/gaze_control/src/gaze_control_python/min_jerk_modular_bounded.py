#!/usr/bin/env python
import rospy
import modern_robotics as mr
import numpy as np

'''
TODO:
	- Bound the velocity/acceleration
'''

'''
Questions:
'''

### Class Waypoint
#		A Waypoint is defined by position, velocity, acceleration
#			and time to get to this point from the previous point
#		The three movement attributes will be stored in a numpy array
#		Dt must be a positive number
class Waypoint():
	def __init__(self, x, dot_x, ddot_x, Dt):
		self.s = np.array([x, dot_x, ddot_x])
		self.Dt = Dt


### Class MinimumJerk
#		Declare this class with a standard vertical array of waypoints
#		Note that in our implementation of Minimum Jerk, Dt takes a different role from in Waypoint
#		It becomes absolute time rather than a change in time, starting from to = 0
class MinimumJerk():
	def __init__(self, waypoint_list, max_vel = 0.5, max_accel = 0.1):
		self.max_vel = max_vel
		self.max_accel = max_accel
		self.waypoint_list = waypoint_list

		# Transfers changes in time between points to absolute time
		# Note that we do i starting at 1 under the assumption that the first point time shift should always be 0
		for i in range(1, len(waypoint_list)):
			self.waypoint_list[i].Dt = waypoint_list[i-1].Dt + waypoint_list[i].Dt

		# Automatically calculate all coefficients
		self.coeffs = np.zeros((len(waypoint_list), 6)) # coeffs for x,y,z axes
		self.get_all_min_jerk_coeffs()


	def bound_this_vel(self, des_vel):
		desired_vel = des_vel
		if (des_vel > self.max_vel):
			desired_vel = self.max_vel
		return desired_vel

		
	
	# Function: Calculates the minimum jerk trajectory between two points
	# Input: two Waypoint classes, initial point then final point
	# Return: coefficient matrix of the 5th order minimum Jerk equation
	def point_min_jerk_coeffs(self, waypoint_i, waypoint_f, axis=0):
		to = waypoint_i.Dt
		tf = waypoint_f.Dt
		T_matrix_coeffs = np.array([ 
			[1, to, to**2,   to**3,       to**4,      to**5 ],
			[0,  1,  2*to, 3*(to**2), 4*(to**3),   5*(to**4)],
			[0,  0,     2,    6*(to), 12*(to**2), 20*(to**3)],
			[1, tf, tf**2,   tf**3,       tf**4,      tf**5 ],
			[0,  1,  2*tf, 3*(tf**2), 4*(tf**3),   5*(tf**4)],
			[0,  0,     2,    6*(tf), 12*(tf**2), 20*(tf**3)],
			])

		boundary_conds = np.array([waypoint_i.s[0], waypoint_i.s[1], waypoint_i.s[2], waypoint_f.s[0], waypoint_f.s[1], waypoint_f.s[2]])	
		coeffs = (np.linalg.inv(T_matrix_coeffs)).dot(boundary_conds)

		return coeffs

	# Function: Updates coeffs Class variable will minimum jerk trajectories for all points
	# Input: None
	# Return: None
	def get_all_min_jerk_coeffs(self, DT_des=1):
		num_waypoints = len(self.waypoint_list)
		for i in range(1, len(self.waypoint_list)):
			self.coeffs[i-1] = self.point_min_jerk_coeffs(self.waypoint_list[i-1], self.waypoint_list[i])

	 
	# Function: Returns the position, velocty, and acceleration of the functions given a time
	# Input: time
	# Return: Numpy array of [position, velocity, acceleration]
	def get_all_s(self, time):
		length = len(self.waypoint_list)
		if(time < 0):
			return None
		elif(time > self.waypoint_list[length-1].Dt):
			return np.array([self.waypoint_list[length-1].s[0], self.waypoint_list[length-1].s[1], self.waypoint_list[length-1].s[2]])
		else:
			for i in range(1, length):
				if (self.waypoint_list[i-1].Dt <= time <= self.waypoint_list[i].Dt):
					# This is ugly but is essentially our 5th order equation
					return np.array([ 
							(self.coeffs[i-1][0] +
						    self.coeffs[i-1][1] * time +
						    self.coeffs[i-1][2] * (time**2) +
						    self.coeffs[i-1][3] * (time**3) +
						    self.coeffs[i-1][4] * (time**4) +
						    self.coeffs[i-1][5] * (time**5)	)
							,
							(self.coeffs[i-1][1] +
						    self.coeffs[i-1][2] * 2 * (time) +
						    self.coeffs[i-1][3] * 3 * (time**2) +
						    self.coeffs[i-1][4] * 4 * (time**3) +
						    self.coeffs[i-1][5] * 5 * (time**4)	)
						    ,
						    (self.coeffs[i-1][2] * 2 +
						    self.coeffs[i-1][3] * 6 * (time) +
						    self.coeffs[i-1][4] * 12 * (time**2) +
						    self.coeffs[i-1][5] * 20 * (time**3))
						    ])

	# Helper functions in case you only want one value
	def get_position(self, time):
		return (self.get_all_s(time))[0]

	def get_velocity(self, time):
		return (self.get_all_s(time))[1]

	def get_acceleration(self, time):
		return (self.get_all_s(time))[2]

# Sample Code:
f = open('output.txt', 'w')
# First define the points as an array pf Waypoint classes
x_coord = []
x_coord.append(Waypoint(65, 65, 4, 0))
x_coord.append(Waypoint(12, 12, 65, 1))
x_coord.append(Waypoint(54, 73, -51, 1))
x_coord.append(Waypoint(78, -87, 7, 1))
x_coord.append(Waypoint(43, 12, -16, 1))
x_coord.append(Waypoint(61, 93, -5, 1))

# Declare a MinimumJerk Class based on the previous array
test_move = MinimumJerk(x_coord)

# Coefficients are automatically calculated, so we can simply input a time
val = np.arange(0, 5.05, 0.05)
for i in range(0, 101):
	f.write( str((test_move.get_all_s(val[i]))[0]) )
	f.write("\t")
	f.write( str((test_move.get_all_s(val[i]))[1]) )
	f.write("\t")
	f.write( str((test_move.get_all_s(val[i]))[2]) )
	f.write("\n")
