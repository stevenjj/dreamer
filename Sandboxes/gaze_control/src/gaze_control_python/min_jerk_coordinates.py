import min_jerk_single as single
import numpy as np
import head_kinematics as hk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

### Class Coordinates_3D
# Coordinates_3D are defined by 3 MinimumJerk classes
# These minimum jerks can be in either polar or cartesian
# The sys variable allows you to specify which one ('p' or 'c')
# Coordinates_3D are done as [x, y, z] <-> [r, theta, phi]
class Coordinates_3D():
	def __init__(self, x, y, z, sys = 'c'):
		self.x = x
		self.y = y
		self.z = z
		self.sys = sys

	# Function: Finds the cartesian coordinates for a given time
	# Input: time
	# Output: np array of [x,y,z] coordinates, translated if the coordinate system is polar
	def get_position(self, time):
		if (self.sys == 'p'):
			# x = r * sin(phi) * cos(theta)
			x = self.x.get_position(time)*np.sin(self.z.get_position(time))*np.cos(self.y.get_position(time))
			# y = r * sin(phi) * sin(theta)
			y = self.x.get_position(time)*np.sin(self.z.get_position(time))*np.sin(self.y.get_position(time))
			# z = r * sin(phi)
			z = self.x.get_position(time)*np.sin(self.z.get_position(time))
			return np.array([x,y,z])
		else:
			return np.array([self.x.get_position(time), self.y.get_position(time), self.z.get_position(time)])

	def get_velocity(self, time):
		return np.array([self.x.get_velocity(time), self.y.get_velocity(time), self.z.get_velocity(time)])
	
	def get_acceleration(self, time):
		return np.array([self.x.get_acceleration(time), self.y.get_acceleration(time), self.z.get_acceleration(time)])

	# In theory all runtimes are the same
	def total_run_time(self):
		return self.x.total_run_time()

	# Tells whether or not to pull the head during a waypoint iteration
	def get_pull(self, time):
		return np.array([self.x.get_pull(time), self.y.get_pull(time), self.z.get_pull(time)])

	def get_special(self, time):
		return self.x.get_special(time)

# Function: Traces a circle in the yz plane with x at a specified distance away
# Inputs: Radius of circle, total time of the trace, optional: distance away from head
# Returns: Minimum Jerk piecewise function for the circle
def circle_yz(radius, time, distance = 1.0):
	accuracy = 100.0
	x = []
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0.0, 0.0, 0.0))
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0.0, 0.0, time))

	y = []
	z = []

	y.append(single.Waypoint(-1.0 * radius, 0.0, 0.0, 0.0))
	z.append(single.Waypoint(0.0 + hk.Head_Kinematics().l1, 0.0, 0.0, 0.0))

	for i in range(1, int(accuracy)):
		theta = np.pi*float(i)/(accuracy/2.0)
		y.append(single.Waypoint(-1.0 * radius * np.cos(theta), 0, 0, time/accuracy ))
		z.append(single.Waypoint(radius * np.sin(theta)+hk.Head_Kinematics().l1, 0, 0, time/accuracy))

	y.append(single.Waypoint(-1.0 * radius, 0.0, 0.0, time/accuracy))
	z.append(single.Waypoint(0.0 + hk.Head_Kinematics().l1, 0.0, 0.0, time/accuracy))
	
	
	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	circle_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return circle_min

# Function: Traces a circle in the xz plane with y at 0
# Inputs: Radius of circle, total time of the trace, optional: distance away from head
# Returns: Minimum Jerk piecewise function for the circle
def circle_xz(radius, time, distance = 1.0):
	accuracy = 64.0
	y = []
	y.append(single.Waypoint(0, 0, 0, 0))
	for i in range(0, int(accuracy)):
		y.append(single.Waypoint(0, 0, 0, time/accuracy))

	x = []
	z = []

	x.append(single.Waypoint(-1 * radius + (hk.Head_Kinematics().l2+distance) , 0, 0, 0))
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, 0))
	for i in range(1, int(accuracy)):
		theta = np.pi*i/(accuracy/2.0)
		x.append(single.Waypoint(-1 * radius * np.cos(theta) + (hk.Head_Kinematics().l2+distance) , 0, 0, time/accuracy ))
		z.append(single.Waypoint(radius*np.sin(theta)+hk.Head_Kinematics().l1, 0, 0, time/accuracy))
	x.append(single.Waypoint(-1 * radius * np.cos(2*np.pi) + (hk.Head_Kinematics().l2+distance) , 0, 0, time/accuracy))
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, time/accuracy))
	
	
	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	circle_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return circle_min

def clover(radius, time, distance = 1):
	accuracy = 64.0
	x = []
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, 0))
	for i in range(0, int(accuracy)):
		x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, time/16.0))

	y = []
	z = []
	y.append(single.Waypoint(0, 0, 0, 0))
	z.append(single.Waypoint(0, 0, 0, 0))

	for i in range(1, int(accuracy)):
		theta = np.pi*i/(accuracy/2.0)
		y.append(single.Waypoint(radius*np.sin(2*theta)*np.cos(theta), 0,  0, time/accuracy))
		z.append(single.Waypoint(radius*np.sin(2*theta)*np.sin(theta), 0, 0, time/accuracy))
	y.append(single.Waypoint(0, 0, 0, time/accuracy))
	z.append(single.Waypoint(0, 0, 0, time/accuracy))


	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	clover_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return clover_min


# Function: Does whatever I need it to for testing purposes
def test_script():
	head_min_jerk = None
	eyes_min_jerk = None
	time = 6.0
	y_dist = .55
	z_dist = .6
	accuracy = 100
	head_x = []
	head_x.append(single.Waypoint(1.2, 0, 0, 0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/16, False, np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, -np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, -np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, -np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, np.pi/12.0))
	head_x.append(single.Waypoint(1.2, 0, 0, time/8, False, -np.pi/12.0))
	
	# head_x.append(single.Waypoint(1.2, 0, 0, time/2))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/3, False, -np.pi/12.0))
	
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(0, 0, 0, time))
	
	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))

	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	head_min = Coordinates_3D(x_coord, y_coord, z_coord)

	eyes_x = []
	eyes_y = []
	eyes_z = []
	eyes_x.append(single.Waypoint(3.0, 0, 0, 0))
	eyes_x.append(single.Waypoint(3.0, 0, 0, time))
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_y.append(single.Waypoint(.5, 0, 0, time/8))
	eyes_y.append(single.Waypoint(.5, 0, 0, time/4))
	eyes_y.append(single.Waypoint(-.5, 0, 0, time/8))
	eyes_y.append(single.Waypoint(-.5, 0, 0, time/4))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1+1.0, 0, 0, time/8.0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1+1.0, 0, 0, time-time/8.0))

	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)

	eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)

	return head_min, eyes_min


# ------------------------------- Actual Behaviors ------------------------------- 

# Function: Causes dreamer to look surprised, then shake head
# Inputs: Eye gaze length in meters
# Outputs: Minimum Jerk functions for head and eyes
# Notes:	Assumes a home configuration
#			TODO: Does not return to perfect home configuration
#			Motion order:
#						 1. Initial gaze point
#						 2. Head back
#						 3. Shake Left
#						 4. Shake Right
#						 5. Shake Left
#						 6. Shake Right
#						 7. Return to center
#						 8. Return to Initial gaze point

def squares_task(head_square_size_in = 0.3):
	head_gaze_length = 0.7
	head_square_size = head_square_size_in

	eye_gaze_length = 0.7
	eye_square_size = 0.1	

	head_traj_duration = 4.0
	eye_traj_duration = 2.0	

	#x, dot_x, ddot_x, Dt

	head_x = []
	head_y = []
	head_z = []

	eyes_x = []
	eyes_y = []
	eyes_z = []

	# Init Gaze Dir
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, 0))
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, 0))
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))


	# Starting Point
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, head_traj_duration))
	head_y.append(single.Waypoint(head_square_size/2.0, 0, 0, head_traj_duration))
	head_z.append(single.Waypoint(head_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, head_traj_duration))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, eye_traj_duration))
	eyes_y.append(single.Waypoint(eye_square_size/2.0, 0, 0, eye_traj_duration))
	eyes_z.append(single.Waypoint(eye_square_size/2.0 + + hk.Head_Kinematics().l1, 0, 0, eye_traj_duration))

	# Waypoint 1
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, head_traj_duration))
	head_y.append(single.Waypoint(head_square_size/2.0, 0, 0, head_traj_duration))
	head_z.append(single.Waypoint(-head_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, head_traj_duration))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, eye_traj_duration))
	eyes_y.append(single.Waypoint(-eye_square_size/2.0, 0, 0, eye_traj_duration))
	eyes_z.append(single.Waypoint(eye_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, eye_traj_duration))

	# Waypoint 2
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, head_traj_duration))
	head_y.append(single.Waypoint(-head_square_size/2.0, 0, 0, head_traj_duration))
	head_z.append(single.Waypoint(-head_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, head_traj_duration))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, eye_traj_duration))
	eyes_y.append(single.Waypoint(-eye_square_size/2.0, 0, 0, eye_traj_duration))
	eyes_z.append(single.Waypoint(-eye_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, eye_traj_duration))

	# Waypoint 3
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, head_traj_duration))
	head_y.append(single.Waypoint(-head_square_size/2.0, 0, 0, head_traj_duration))
	head_z.append(single.Waypoint(head_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, head_traj_duration))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, eye_traj_duration))
	eyes_y.append(single.Waypoint(eye_square_size/2.0, 0, 0, eye_traj_duration))
	eyes_z.append(single.Waypoint(-eye_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, eye_traj_duration))

	# Waypoint 4
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, head_traj_duration))
	head_y.append(single.Waypoint(head_square_size/2.0, 0, 0, head_traj_duration))
	head_z.append(single.Waypoint(head_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, head_traj_duration))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, eye_traj_duration))
	eyes_y.append(single.Waypoint(eye_square_size/2.0, 0, 0, eye_traj_duration))
	eyes_z.append(single.Waypoint(eye_square_size/2.0 + hk.Head_Kinematics().l1, 0, 0, eye_traj_duration))


	# Waypoint 5
	head_x.append(single.Waypoint(head_gaze_length, 0,  0, head_traj_duration))
	head_y.append(single.Waypoint(0, 0, 0, head_traj_duration))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, head_traj_duration))

	eyes_x.append(single.Waypoint(eye_gaze_length, 0, 0, eye_traj_duration))
	eyes_y.append(single.Waypoint(0, 0, 0, eye_traj_duration))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, eye_traj_duration))


	head_x_coord = single.MinimumJerk(head_x)
	head_y_coord = single.MinimumJerk(head_y)
	head_z_coord = single.MinimumJerk(head_z)

	eyes_x_coord = single.MinimumJerk(eyes_x)
	eyes_y_coord = single.MinimumJerk(eyes_y)
	eyes_z_coord = single.MinimumJerk(eyes_z)

	head_traj = Coordinates_3D(head_x_coord, head_y_coord, head_z_coord)
	eye_traj = Coordinates_3D(eyes_x_coord, eyes_y_coord, eyes_z_coord)

	return head_traj, eye_traj


def surprised_no(gaze_length = 1.0):
	head_min_jerk = None
	eyes_min_jerk = None
	time = 7.5
	no_distance = gaze_length / np.cos(np.pi/6.0) # Causes 30 degree motion regardless of focus point
	# 60% of maxmimum joint movement
	head_back = 0 # np.sin(.26)*hk.Head_Kinematics().l1*.6
	# Head will move according to above
	head_x = []
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 0))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2 - head_back, 0, 0, .25, True))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2 - head_back, 0, 0, 5.25))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 2.0, True))

	
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(0, 0, 0, 1.5))
	head_y.append(single.Waypoint(no_distance, 0, 0, .25))
	head_y.append(single.Waypoint(-no_distance, 0, 0, .5))
	head_y.append(single.Waypoint(no_distance, 0, 0, .5))
	head_y.append(single.Waypoint(-no_distance, 0, 0, .5))
	head_y.append(single.Waypoint(0, 0, 0, .25))
	head_y.append(single.Waypoint(0, 0, 0, 4.0))
	
	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1 + head_back * 3, 0, 0, .25))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1 + head_back * 3, 0, 0, 5.25))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 2.0))
	

	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	surprised_no_head_min = Coordinates_3D(x_coord, y_coord, z_coord)
	
	


	eyes_x = []
	eyes_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 0))
	eyes_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, time))
	eyes_y = []
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_y.append(single.Waypoint(0, 0, 0, time))
	eyes_z = []
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))

	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)

	surprised_no_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)

	return surprised_no_head_min, surprised_no_eyes_min


# Function: Looks away and rolls eyes
# Inputs: x gaze length
# Returns: head and eye min_jerk
# Notes:	Assumes a home configuration, eye priority motion
#			Probably robot agnostic
#			Standard x gaze length for a human of eye width 6cm is considered to be 80cm
#				This parameter is changable, but for robot agnostic behaviors, we can simply scale from the distance between the robot's eyes
#			Equation for head motion: z is linear with respect to y and x remains constant
#			Equation for eye motion: x and y travel in a elliptical shape while z is a parabola
#			Motion order:
#						 1. Initial gaze point forward 
#						 2a. Tilt head slightly while looking up to the right
#						 2b. Eyes roll in parabola form
def roll_eyes(x_length = .8):
	head_min_jerk = None
	eyes_min_jerk = None
	time = 1.9
	accuracy = 150

	x_gaze_length = hk.Head_Kinematics().l2 + x_length

	# Apply tilting to head x coordinate
	head_x = []
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, False, np.pi/12.0))
	
	# We want a pi/6 rotation of the head about the z axis which will be dependent upon gaze length
	# z head gaze will follow a 1-1 ratio
	y_gaze_length = -x_gaze_length * np.tan(np.pi/6.0)
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(y_gaze_length, 0, 0, time))

	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	head_z.append(single.Waypoint(-y_gaze_length + hk.Head_Kinematics().l1, 0, 0, time))


	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	roll_eyes_head_min = Coordinates_3D(x_coord, y_coord, z_coord)
	

	eyes_x = []
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))

	eyes_y = []
	eyes_y.append(single.Waypoint(0, 0, 0, 0))

	eyes_z = []
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	# eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))
	stop_point = float(-y_gaze_length)

	# Multiplier for how much higher above the head position to look
	# Human max gaze up is 30 degrees so because the head already moves 30 degrees up, we simply double
	peak_mult = 2
	
	# Scaling factor for the parabola so that the left bound is always at hk.Head_Kinematics().l1
	scale = (peak_mult*y_gaze_length + hk.Head_Kinematics().l1)/(y_gaze_length**2)
	
	# Find where the parabola end point is. np.pi/3 radians from the inition point respective to the y axis
	a = 20 # Arbitrary value greater than the check
	while(a > (np.pi/3.0) ):
		a = np.arctan( (peak_mult*-y_gaze_length + scale*((stop_point+y_gaze_length)**2) - hk.Head_Kinematics().l1) / stop_point)
		stop_point+=.001 # 1mm of accuracy


	time_range = np.linspace(0, stop_point, accuracy)
	for i in range(accuracy):
		t = time_range[i]
		eyes_x.append(single.Waypoint(x_gaze_length*np.cos(i*np.pi/(6.0*accuracy)), 0, 0, time/float(accuracy)))
		eyes_y.append(single.Waypoint(-2*x_gaze_length*np.sin(i*np.pi/(6.0*accuracy)), 0, 0, time/float(accuracy)))
		eyes_z.append(single.Waypoint(peak_mult*-y_gaze_length + scale*((t+y_gaze_length)**2), 0, 0, time/float(accuracy)))
	
	
	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)
	roll_eyes_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)

	return roll_eyes_head_min, roll_eyes_eyes_min


class piecewise_np():
	def to_array(self, t):
		x = [t]
		x = np.array(x)
		return x

	def dt_to_t(self, dt):
		current = 0.0
		t_return = []
		for i in range( len(dt) ):
			current += dt[i]
			t_return.append(current)
		return t_return

	def roll_eyes(self, t):
		t = self.to_array(t)
		dt = [0, 5, 5, 1]
		times = self.dt_to_t(dt)
		self.total_run_time = times[len(times)-1]
		x = np.piecewise(t, 
						[ (times[0] <= t) & (t < times[1]),
						(times[1]<= t) & ( t<= times[2])],

						[lambda t: t, lambda t: t])
		return x[0]
	
	behavior_dictionary = {1:roll_eyes}
	def __init__(self, behavior_num):
		self.behavior = self.behavior_dictionary[behavior_num]
		self.total_run_time = 0

	def get_position(self, t):
		return self.behavior(self, t)


	def total_run_time(self):
		self.behavior(0)
		return self.total_run_time


'''
a, b = test_script()
time = a.total_run_time()
x = np.linspace(0, time, 200)
y=[]
for i in range(0, 200):
	y.append(a.get_special(x[i]))
plt.plot(x, y)
plt.show()
'''
'''
a = piecewise_np(1)


# coordinate1 = circle_xz(.80, 16.0)
coordinate1, coordinate2 = roll_eyes()

count = 0
x = []
y = []
z = []
while count < coordinate2.total_run_time():
    x.append(coordinate2.get_position(count)[0])
    y.append(coordinate2.get_position(count)[1])
    z.append(coordinate2.get_position(count)[2])
    count = count + .05
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z)

# coordinate1, coordinate2 = roll_eyes(.5)
# count = 0
# x = []
# y = []
# z = []
# while count < coordinate2.total_run_time():
#     x.append(coordinate2.get_position(count)[0])
#     y.append(coordinate2.get_position(count)[1])
#     z.append(coordinate2.get_position(count)[2])
#     count = count + .05
# ax.plot(x, y, z)
plt.show()
'''
