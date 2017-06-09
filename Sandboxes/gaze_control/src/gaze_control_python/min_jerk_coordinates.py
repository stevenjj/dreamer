import min_jerk_single as single
import numpy as np
import head_kinematics as hk

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

	# In theory all runtimes are the same
	def total_run_time(self):
		return self.x.total_run_time()

	# Tells whether or not to pull the head during a waypoint iteration
	def get_pull(self, time):
		return np.array([self.x.get_pull(time), self.y.get_pull(time), self.z.get_pull(time)])

	def get_tilt(self, time):
		return self.x.get_tilt(time)

# Function: Traces a circle in the yz plane with x at a specified distance away
# Inputs: Radius of circle, total time of the trace, optional: distance away from head
# Returns: Minimum Jerk piecewise function for the circle
def circle_yz(radius, time, distance = 1):
	accuracy = 16.0
	x = []
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, 0))
	for i in range(0, int(accuracy)):
	    x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, time/accuracy))

	y = []
	z = []

	y.append(single.Waypoint(-1 * radius, 0, 0, 0))
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, 0))
	for i in range(1, int(accuracy)):
		theta = np.pi*i/(accuracy/2.0)
		y.append(single.Waypoint(-1 * radius * np.cos(theta), radius * np.sin(theta), radius * np.cos(theta), time/accuracy ))
		z.append(single.Waypoint(radius*np.sin(theta)+hk.Head_Kinematics().l1, radius*np.cos(theta), -1*radius*np.sin(theta), time/accuracy))
	y.append(single.Waypoint(-1 * radius * np.cos(2*np.pi), 0, 0, time/accuracy))
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, time/accuracy))
	
	
	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	circle_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return circle_min

# Function: Traces a circle in the xz plane with y at 0
# Inputs: Radius of circle, total time of the trace, optional: distance away from head
# Returns: Minimum Jerk piecewise function for the circle
def circle_xz(radius, time, distance = 1):
	accuracy = 16.0
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
		x.append(single.Waypoint(-1 * radius * np.cos(theta) + (hk.Head_Kinematics().l2+distance) , radius * np.sin(theta), radius * np.cos(theta), time/accuracy, True ))
		z.append(single.Waypoint(radius*np.sin(theta)+hk.Head_Kinematics().l1, radius*np.cos(theta), -1*radius*np.sin(theta), time/accuracy))
	x.append(single.Waypoint(-1 * radius * np.cos(2*np.pi) + (hk.Head_Kinematics().l2+distance) , 0, 0, time/accuracy,True))
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, time/accuracy))
	
	
	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	circle_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return circle_min

def clover(radius, time, distance = 1):
	accuracy = 32.0
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
		y.append(single.Waypoint(radius*np.sin(2*theta)*np.cos(theta), radius*.5*(np.cos(theta) + 3*np.cos(3*theta)),  radius*(-.5*np.sin(theta)-4.5*np.sin(3*theta)), time/accuracy))
		z.append(single.Waypoint(radius*np.sin(2*theta)*np.sin(theta), radius*.5*(3*np.sin(3*theta)-np.sin(theta)), radius*.5*(9*np.cos(3*theta)-np.cos(theta)), time/accuracy))
	y.append(single.Waypoint(0, 0, 0, time/accuracy))
	z.append(single.Waypoint(0, 0, 0, time/accuracy))


	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	clover_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return clover_min

# Function: Does whatever I need it to for testing purposes
def test_script(tilt):
	head_min_jerk = None
	eyes_min_jerk = None
	time = 8.0
	head_x = []
	head_x.append(single.Waypoint(.5+hk.Head_Kinematics().l2, 0, 0, 0))
	head_x.append(single.Waypoint(1+hk.Head_Kinematics().l2, 0, 0, time, False, tilt))
	
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(.5, 0, 0, time))
	
	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1+.5, 0, 0, time))

	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	head_min = Coordinates_3D(x_coord, y_coord, z_coord)

	eyes_x = []
	eyes_y = []
	eyes_z = []

	return head_min


# ------------------------------- Actual Behaviors ------------------------------- 



# Function: Causes dreamer to look surprised, then shake head
# Inputs: Eye gaze length in meters
# Outputs: Minimum Jerk functions for head and eyes
# Notes:	Assumes a home configuration
#			Motion order:
# 						1. Initial gaze point
# 						2. Head back
# 						3. Shake Left
# 						4. Shake Right
# 						5. Shake Left
# 						6. Shake Right
# 						7. Return to center
# 						8. Return to Initial gaze point
def surprised_no(gaze_length):
	head_min_jerk = None
	eyes_min_jerk = None
	time = 7.0
	no_distance = .3
	head_back = .04
	# Head will move according to above
	head_x = []
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 0))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2 - head_back, 0, 0, 1.0, True))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2 - head_back, 0, 0, 5.0))
	head_x.append(single.Waypoint(gaze_length + hk.Head_Kinematics().l2, 0, 0, 1.0, True))
	
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(0, 0, 0, 2.0))
	head_y.append(single.Waypoint(no_distance, 0, 0, .25))
	head_y.append(single.Waypoint(-no_distance, 0, 0, .5))
	head_y.append(single.Waypoint(no_distance, 0, 0, .5))
	head_y.append(single.Waypoint(-no_distance, 0, 0, .5))
	head_y.append(single.Waypoint(0, 0, 0, .25))
	head_y.append(single.Waypoint(0, 0, 0, 3.0))
	
	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1 + head_back*4, 0, 0, 1.0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1 + head_back*4, 0, 0, 5.0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 1.0))
	

	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	surprised_no_head_min = Coordinates_3D(x_coord, y_coord, z_coord)
	
	


	eyes_x = []
	eyes_x.append(single.Waypoint(gaze_length*2 +hk.Head_Kinematics().l2, 0, 0, 0))
	eyes_x.append(single.Waypoint(gaze_length*2 +hk.Head_Kinematics().l2, 0, 0, time))
	eyes_y = []
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_y.append(single.Waypoint(0, 0, 0, time))
	eyes_z = []
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1+head_back, 0, 0, time))

	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)

	surprised_no_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)

	return surprised_no_head_min, surprised_no_eyes_min






# f = open('output.txt', 'w')

# coordinate1, coordinates2 = surprised_no(1)
# print coordinate1.total_run_time()



# val = np.arange(0, 8.01, 0.01)
# ran = 800
# for i in range(0, ran):
# 	f.write(str(coordinate1.get_position(val[i])[0]))
# 	f.write('\t')
# 	f.write(str(coordinate1.get_position(val[i])[1]))
# 	f.write('\t')
# 	f.write(str(coordinate1.get_position(val[i])[2]))
# 	f.write('\n')

# f.write("x = [")
# for i in range(0, ran):
# 	f.write( str((coordinate1.get_velocity(val[i]))[0]) )
# 	f.write(", ")
# f.write("]\n")

# f.write("y = [")
# for i in range(0, ran):
# 	f.write( str((coordinate1.get_velocity(val[i]))[1]) )
# 	f.write(", ")
# f.write("]\n")

# f.write("z = [")
# for i in range(0, ran):
# 	f.write( str((coordinate1.get_velocity(val[i]))[2]) )
# 	f.write(", ")
# f.write("]")

'''
	f.write('\t')
	f.write('\t')
'''
# x = []
# x.append(single.Waypoint(1, 0, 0, 0))
# x.append(single.Waypoint(1, 0, 0, 1))
# x.append(single.Waypoint(1, 0, 0, 1))
# x.append(single.Waypoint(1, 0, 0, 1))
# x.append(single.Waypoint(1, 0, 0, 1))

# y = []
# y.append(single.Waypoint(-.95, 0, 0, 0))
# y.append(single.Waypoint(.95, .05, -.05, 1))
# y.append(single.Waypoint(.95, .05, .05, 1))
# y.append(single.Waypoint(-.95, .05, -.05, 1))
# y.append(single.Waypoint(-.95, 0, 0, 1))

# z = []
# z.append(single.Waypoint(.05, 0, 0, 0))
# z.append(single.Waypoint(.05, .05, .05, 1))
# z.append(single.Waypoint(.95, .05, -.05, 1))
# z.append(single.Waypoint(.95, .05, -.05, 1))
# z.append(single.Waypoint(.05, 0, 0, 1))

# x_coord = single.MinimumJerk(x)
# y_coord = single.MinimumJerk(y)
# z_coord = single.MinimumJerk(z)

# coordinate1 = Coordinates_3D(x_coord, y_coord, z_coord)