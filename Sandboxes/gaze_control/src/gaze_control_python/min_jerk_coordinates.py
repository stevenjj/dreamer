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
			# z = r * cos(phi) + head link height
			z = self.x.get_position(time)*np.cos(self.z.get_position(time))+hk.Head_Kinematics().l1
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
		return self.x.get_pull(time)

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
	gaze_length = 3.5
	shift = .3
	yaw = .3
	time = 6.0
	head_x = []
	head_x.append(single.Waypoint(gaze_length, 0, 0, 0))
	head_x.append(single.Waypoint(gaze_length, 0, 0, time))
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
	eyes_x.append(single.Waypoint(gaze_length, 0, 0, 0))
	eyes_x.append(single.Waypoint(gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_y.append(single.Waypoint(0, 0, 0, time))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))
	
	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)
	eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)
	

	# y_dist = .55
	# z_dist = .6
	# accuracy = 100
	# head_x = []
	# head_x.append(single.Waypoint(1.2, 0, 0, 0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/16, np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, -np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, -np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, -np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, np.pi/12.0))
	# head_x.append(single.Waypoint(1.2, 0, 0, time/8, -np.pi/12.0))
	
	# # head_x.append(single.Waypoint(1.2, 0, 0, time/2))
	# # head_x.append(single.Waypoint(1.2, 0, 0, time/3, False, -np.pi/12.0))
	
	# head_y = []
	# head_y.append(single.Waypoint(0, 0, 0, 0))
	# head_y.append(single.Waypoint(0, 0, 0, time))
	
	# head_z = []
	# head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	# head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))

	# x_coord = single.MinimumJerk(head_x)
	# y_coord = single.MinimumJerk(head_y)
	# z_coord = single.MinimumJerk(head_z)

	# head_min = Coordinates_3D(x_coord, y_coord, z_coord)

	# eyes_x = []
	# eyes_y = []
	# eyes_z = []
	# eyes_x.append(single.Waypoint(3.0, 0, 0, 0))
	# eyes_x.append(single.Waypoint(3.0, 0, 0, time))
	# eyes_y.append(single.Waypoint(0, 0, 0, 0))
	# eyes_y.append(single.Waypoint(0, 0, 0, time))
	# eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	# eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))

	# x_coord = single.MinimumJerk(eyes_x)
	# y_coord = single.MinimumJerk(eyes_y)
	# z_coord = single.MinimumJerk(eyes_z)

	# eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)

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
def surprised_no(gaze_length = 1.0):
	head_min_jerk = None
	eyes_min_jerk = None
	time = 7.5
	no_distance = gaze_length * np.tan(np.pi/12.0) # Causes 30 degree motion total regardless of focus point
	# 60% of maxmimum joint movement
	head_back = 0 # np.sin(.26)*hk.Head_Kinematics().l1*.6
	# Head will move according to above
	head_x = []
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 0))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2 - head_back, 0, 0, .25, 0, True))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2 - head_back, 0, 0, 5.25))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 2.0, 0, True))

	
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
	time = 2.40
	accuracy = 150


	static_x_gaze_length = hk.Head_Kinematics().l2 + .8
	# Apply tilting to head x coordinate
	head_x = []
	head_x.append(single.Waypoint(static_x_gaze_length, 0, 0, 0))
	head_x.append(single.Waypoint(static_x_gaze_length, 0, 0, time, -np.pi/26.0))
	
	# We want a pi/6 rotation of the head about the z axis which will be dependent upon gaze length
	# z head gaze will follow a 1-1 ratio
	static_y_gaze_length = -static_x_gaze_length * np.tan(np.pi/5.0)
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(0, 0, 0, time))
	head_z = []
	head_z.append(single.Waypoint(np.pi/2.0 - np.pi/45.0, 0, 0, 0))
	head_z.append(single.Waypoint(np.pi/2.0 - np.pi/45.0, 0, 0, time))


	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	roll_eyes_head_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')
	


	# Eyes are done in polar coordinates
	# x is radius, y is theta, z is phi

	x_gaze_length = hk.Head_Kinematics().l2 + x_length
	y_gaze_length = -x_gaze_length * np.tan(np.pi/6.0)

	eyes_x = []
	eyes_y = []
	eyes_z = []
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))


	eyes_y.append(single.Waypoint(np.pi/12.0, 0, 0, 0))
	for i in range(1, accuracy):
		t = i/float(accuracy)
		eyes_y.append(single.Waypoint((-np.pi/7.0 - np.pi/12.0)*t + np.pi/12.0, 0, 0, time/accuracy))



	start_z = 3*np.pi/5.0
	peak_z = np.pi/3.0

	eyes_z.append(single.Waypoint(start_z, 0, 0, 0))
	for i in range(1, accuracy):
		t = .85*i*np.pi/float(accuracy)
		eyes_z.append(single.Waypoint((peak_z-start_z)*np.sin(t) + start_z, 0, 0, time/accuracy))
	

	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)
	roll_eyes_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')

	return roll_eyes_head_min, roll_eyes_eyes_min
	
	
# Function: Makes dreamer look sleepy, needs eyelids added though
# Inputs: None
# Returns: head and eye min_jerk
# Notes:
#		pi/13~ is the max I can roll
# 		Done in polar coordinates
# 		Motion Order:
#			1. random motion up and down with faster motion up than down
# 			
# 
# 
def sleepy():
	x_gaze_length = 1.0
	accuracy = 25
	close_eyes = np.pi*35.0/36.0
	open_eyes = np.pi/36.0
	rand_head_pitch_low = (np.random.rand()) * np.pi/18.0 + np.pi/2
	rand_head_yaw = (np.random.rand()-.5) * np.pi/12

	head_x = []
	head_y = []
	head_z = []
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	head_y.append(single.Waypoint(rand_head_yaw, 0, 0, 0))
	head_z.append(single.Waypoint(rand_head_pitch_low, 0, 0, 0))

	eyes_x = []
	eyes_y = []
	eyes_z = []
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	eyes_y.append(single.Waypoint(rand_head_yaw, 0, 0, 0))
	eyes_z.append(single.Waypoint(rand_head_pitch_low, 0, 0, 0))

	for i in range(10):
		rand_head_tilt = (np.random.rand()-.5) * np.pi/8
		rand_head_pitch_low = (np.random.rand()) * np.pi/8.0 + np.pi/2.0 + np.pi/18.0
		rand_head_pitch_high = (np.random.rand()) * -np.pi/36.0 + np.pi/2.0
		rand_head_yaw = (np.random.rand()-.5) * np.pi/12
		time = (np.random.rand() + 1.0) * 2 * (rand_head_pitch_low-np.pi/2.0)/(np.pi/18.0) 
		time2 = (rand_head_pitch_low-np.pi/2.0)/(np.pi/18.0)

		head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, rand_head_tilt))
		head_y.append(single.Waypoint(rand_head_yaw, 0, 0, time))
		head_z.append(single.Waypoint(rand_head_pitch_low, 0, 0, time))
		
		eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time, close_eyes))
		eyes_y.append(single.Waypoint(rand_head_yaw, 0, 0, time))
		eyes_z.append(single.Waypoint(rand_head_pitch_low, 0, 0, time))


		rand_head_yaw_low = (np.random.rand()-.5)*np.pi/12.0
		head_x.append(single.Waypoint(x_gaze_length, 0, 0, time2, rand_head_tilt * .5 * (np.random.rand() - .5)))
		head_y.append(single.Waypoint(rand_head_yaw_low, 0, 0, time2))
		head_z.append(single.Waypoint(rand_head_pitch_high, 0, 0, time2))
		
		eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time2, open_eyes))
		eyes_y.append(single.Waypoint(rand_head_yaw_low, 0, 0, time2))
		eyes_z.append(single.Waypoint(rand_head_pitch_high, 0, 0, time2))


	rand_head_tilt = (np.random.rand()-.5) * np.pi/8
	rand_head_pitch_low = (np.random.rand()) * np.pi/8.0 + np.pi/2.0 + np.pi/18.0
	rand_head_yaw = (np.random.rand()-.5) * np.pi/12
	time = (np.random.rand() + 1.0) * 2 * (rand_head_pitch_low-np.pi/2.0)/(np.pi/18.0) 
	
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, rand_head_tilt))
	head_y.append(single.Waypoint(rand_head_yaw, 0, 0, time))
	head_z.append(single.Waypoint(rand_head_pitch_low, 0, 0, time))
	
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time, close_eyes))
	eyes_y.append(single.Waypoint(rand_head_yaw, 0, 0, time))
	eyes_z.append(single.Waypoint(rand_head_pitch_low, 0, 0, time))


	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)
	sleepy_head_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')
			
	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)
	sleepy_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')

	return sleepy_head_min, sleepy_eyes_min


def ashamed():
	scale = 2.0
	x_gaze_length = .75

	head_pitch = np.pi/2 + np.pi/15.0
	head_yaw = np.pi/6.0

	head_look_pitch = np.pi/2.0 + np.pi/30.0
	head_look_yaw = np.pi/12.0
	eyes_look_pitch = np.pi/2.0 - np.pi/15.0

	head_x = []
	head_y = []
	head_z = []

	eyes_x = []
	eyes_y = []
	eyes_z = []

	# First look down and away
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	head_y.append(single.Waypoint(head_yaw, 0, 0, 0))
	head_z.append(single.Waypoint(head_pitch, 0, 0, 0))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	eyes_y.append(single.Waypoint(head_yaw, 0, 0, 0))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, 0))

	time_hold = 1.5 * scale
	# Hold
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	head_y.append(single.Waypoint(head_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(head_yaw, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))

	time = 2.0 * scale
	# Peek
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(head_look_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_look_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(0, 0, 0, time))
	eyes_z.append(single.Waypoint(eyes_look_pitch, 0, 0, time))

	time_hold = .5 * scale
	# Hold Peek
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	head_y.append(single.Waypoint(head_look_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_look_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(0, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(eyes_look_pitch, 0, 0, time_hold))

	time = 1.25 * scale
	# Move head quickly away again
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(head_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(head_yaw, 0, 0, time))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time))	

	time_hold = 1.0 * scale
	# Hold
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	head_y.append(single.Waypoint(head_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(head_yaw, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))

	time = 1.5 * scale
	# Look away more
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(head_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(head_yaw*1.5, 0, 0, time))
	eyes_z.append(single.Waypoint(head_look_pitch, 0, 0, time))

	time_hold = .85 * scale
	# Hold
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	head_y.append(single.Waypoint(head_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(head_yaw*1.5, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(head_look_pitch, 0, 0, time_hold))

	time = 2.5 * scale
	# Move head to other side 
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(-head_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(-head_yaw*1.3, 0, 0, time))
	eyes_z.append(single.Waypoint(head_pitch+np.pi/12.0, 0, 0, time/2.0))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time/2.0))

	time_hold = 2.0 * scale
	# Hold 
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold, -np.pi/52.0))
	head_y.append(single.Waypoint(-head_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(-head_yaw*1.3, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))
	
	time = 1.75 * scale
	# Look up some 
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(-head_look_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(-head_look_yaw, 0, 0, time))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time))

	time_hold = 2.0 * scale
	# Hold 
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	head_y.append(single.Waypoint(-head_look_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(-head_look_yaw, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time_hold))

	time = 1.5 * scale
	# Peek again 
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(-head_look_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_look_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(0, 0, 0, time))
	eyes_z.append(single.Waypoint(eyes_look_pitch, 0, 0, time))

	time_hold = .80 * scale
	# Hold peek
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	head_y.append(single.Waypoint(-head_look_yaw, 0, 0, time_hold))
	head_z.append(single.Waypoint(head_look_pitch, 0, 0, time_hold))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_hold))
	eyes_y.append(single.Waypoint(0, 0, 0, time_hold))
	eyes_z.append(single.Waypoint(eyes_look_pitch, 0, 0, time_hold))

	time = 1.5 * scale
	# Finally look away
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	head_y.append(single.Waypoint(head_yaw, 0, 0, time))
	head_z.append(single.Waypoint(head_pitch, 0, 0, time))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time))
	eyes_y.append(single.Waypoint(head_yaw, 0, 0, time))
	eyes_z.append(single.Waypoint(head_pitch, 0, 0, time))

	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)
	shy_head_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')
			
	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)
	shy_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')

	return shy_head_min, shy_eyes_min


def judging_no():
	x_gaze_length = 1.0
	theta = 0
	phi = np.pi/2.0
	time_total = 10.0
	tilt = np.pi/18.0
	head_drop = phi + np.pi/15.0
	head_x = []
	head_y = []
	head_z = []

	# Start with head homed
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	head_y.append(single.Waypoint(theta, 0, 0, 0))
	head_z.append(single.Waypoint(phi, 0, 0, 0))

	time = .5
	# Move head up
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, tilt))
	# head_x.append(single.Waypoint(x_gaze_length, 0, 0, time/2.0, tilt))
	head_y.append(single.Waypoint(theta, 0, 0, time))
	head_z.append(single.Waypoint(phi-np.pi/60.0, 0, 0, time))

	time = 1.25
	# Drop head, heavy tilting
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, tilt))
	head_y.append(single.Waypoint(theta + np.pi/15.0, 0, 0, time))
	head_z.append(single.Waypoint(head_drop, 0, 0, time))

	time = 1.75
	# Hold
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, tilt))
	head_y.append(single.Waypoint(theta + np.pi/15.0, 0, 0, time))
	head_z.append(single.Waypoint(head_drop, 0, 0, time))

	time = 4
	# Begin no
	head_x.append(single.Waypoint(x_gaze_length, 0, 0, time, tilt))

	accuracy = 100
	for i in range(accuracy):
		t = 5.5*i*np.pi/accuracy
		head_y.append(single.Waypoint(theta + np.pi/15.0 + (np.pi/20.0) * np.sin(t), 0, 0, time/float(accuracy)))

	head_z.append(single.Waypoint(head_drop, 0, 0, time))
	



	eyes_x = []
	eyes_y = []
	eyes_z = []

	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, 0))
	eyes_x.append(single.Waypoint(x_gaze_length, 0, 0, time_total))
	eyes_y.append(single.Waypoint(theta, 0, 0, 0))
	eyes_y.append(single.Waypoint(theta, 0, 0, time_total))
	eyes_z.append(single.Waypoint(phi, 0, 0, 0))
	eyes_z.append(single.Waypoint(phi, 0, 0, time_total))


	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)
	judging_no_head_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')
			
	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)
	judging_no_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord, 'p')

	return judging_no_head_min, judging_no_eyes_min



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
		x_gaze_length = 1.0
		dt = [0, 2]
		times = self.dt_to_t(dt)
		self.total_run_time = times[len(times)-1]
		head_x = np.piecewise(t, 
						[ (times[0] <= t) & (t < times[1]),
							(times[1] <= t)],

						[x_gaze_length, x_gaze_length])
		head_y = np.piecewise(t, 
						[ (times[0] <= t) & (t < times[1]),
						(times[1] <= t)],

						[0, 0])
		head_z = np.piecewise(t, 
						[ (times[0] <= t) & (t < times[1]),
						(times[1] <= t)],

						[lambda t: t, lambda t: t])

		eyes_y = np.piecewise(t, 
						[ (times[0] <= t) & (t < times[1]),
						(times[1] <= t)],

						[lambda t: (-np.pi/8.0 - np.pi/12.0)*t + np.pi/12.0, 
						(-np.pi/8.0 - np.pi/12.0)*times[1] + np.pi/12.0])
		return head_x[0]
	
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
# coordinate1 = circle_xz(.80, 16.0)
coordinate1, coordinate2 = judging_no()

count = 0
x = []
y = []
z = []
while count < coordinate1.total_run_time():
    x.append(coordinate1.get_position(count)[0])
    y.append(coordinate1.get_position(count)[1])
    z.append(coordinate1.get_position(count)[2])
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
