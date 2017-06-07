#!/usr/bin/env python
import rospy
import numpy as np

### Class Waypoint
#		A Waypoint is defined by position, velocity, acceleration
#			and time to get to this point from the previous point
#		The three movement attributes will be stored in a numpy array
#		Dt must be a positive number
#		pull_head notes that we want that point to move the head itself rather than change the gaze length
class Waypoint():
	def __init__(self, x, dot_x, ddot_x, Dt, pull_head = False):
		self.s = np.array([x, dot_x, ddot_x])
		self.Dt = Dt
		self.pull_head = pull_head

	def get_pull(self):
		return self.pull_head

### Class MinimumJerk
#		Declare this class with a standard vertical array of waypoints
#		Note that in our implementation of Minimum Jerk, Dt takes a different role from in Waypoint
#		It becomes absolute time rather than a change in time, starting from t = 0
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


	def total_run_time(self):
		return self.waypoint_list[len(self.waypoint_list)-1].Dt
	
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

	# Function: Updates coeffs Class variable with minimum jerk trajectories for all points
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
			time = self.waypoint_list[length-1].Dt

		for i in range(1, length):
			if (self.waypoint_list[i-1].Dt <= time <= self.waypoint_list[i].Dt):
				# This is ugly but is essentially our 5th order equation and its derivatives
				return np.array([ # position
						(self.coeffs[i-1][0] +
					    self.coeffs[i-1][1] * time +
					    self.coeffs[i-1][2] * (time**2) +
					    self.coeffs[i-1][3] * (time**3) +
					    self.coeffs[i-1][4] * (time**4) +
					    self.coeffs[i-1][5] * (time**5)	)
						,
						(self.coeffs[i-1][1] + # velocity
					    self.coeffs[i-1][2] * 2 * (time) +
					    self.coeffs[i-1][3] * 3 * (time**2) +
					    self.coeffs[i-1][4] * 4 * (time**3) +
					    self.coeffs[i-1][5] * 5 * (time**4)	)
					    ,
					    (self.coeffs[i-1][2] * 2 + # acceleration
					    self.coeffs[i-1][3] * 6 * (time) +
					    self.coeffs[i-1][4] * 12 * (time**2) +
					    self.coeffs[i-1][5] * 20 * (time**3))
					    ,
						( self.coeffs[i-1][3] * 6 + # jerk
					    self.coeffs[i-1][4] * 24 * (time) +
					    self.coeffs[i-1][5] * 60 * (time**2))
					    ])


	# Helper functions in case you only want one value
	def get_position(self, time):
		return (self.get_all_s(time))[0]

	def get_velocity(self, time):
		return (self.get_all_s(time))[1]

	def get_acceleration(self, time):
		return (self.get_all_s(time))[2]

	def get_jerk(self, time):
		return (self.get_all_s(time))[3]


	def get_pull(self, time):
		length = len(self.waypoint_list)
		if(time < 0):
			return None

		elif(time > self.waypoint_list[length-1].Dt):
			time = self.waypoint_list[length-1].Dt

		for i in range(1, length):
			if (self.waypoint_list[i-1].Dt <= time <= self.waypoint_list[i].Dt):
				return self.waypoint_list[i].get_pull()

'''
### Sample Code:
# Declare an array and append Waypoint Classes
x_coord = []
x_coord.append(Waypoint(0, 0, 0, 0))
x_coord.append(Waypoint(10, 10, 5, 1))
x_coord.append(Waypoint(20, 10, 5, 1))
x_coord.append(Waypoint(30, 10, 5, 1))
x_coord.append(Waypoint(10, 20, 5, 1))
x_coord.append(Waypoint(20, 0, 0, 1))

# Declare a MinimumJerk class from those waypoints
test_move = MinimumJerk(x_coord)
print "Coefficients: ",
print test_move.coeffs
print ""

# Simply input a time to get parameters at any give time after 0
print "[Position, Velocity, Acceleration] @ t = 1"
print test_move.get_all_s(1)
print ""
print "[Position, Velocity, Acceleration] @ t = 1.8"
print test_move.get_all_s(1.8)

print test_move.total_run_time()
'''

# Testing Script
f = open('output.txt', 'w')
x_coord = []
x_coord.append(Waypoint(0, 0, 0, 0))
x_coord.append(Waypoint(10, 0, 0, 1, True))
x_coord.append(Waypoint(5, 0, 0, 1))
x_coord.append(Waypoint(0, 0, 0, 1, True))




# Declare a MinimumJerk Class based on the previous array
test_move = MinimumJerk(x_coord)

# Coefficients are automatically calculated, so we can simply input a time
val = np.arange(0, 3, 0.01)
for i in range(0, 300):
	f.write("Time: ")
	f.write(str(i/100.0))
	f.write("	Pull: ")
	f.write( str(test_move.get_pull(val[i])) )
	f.write("\n")
