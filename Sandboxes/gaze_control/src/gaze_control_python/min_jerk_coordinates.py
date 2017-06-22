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

	def get_tilt(self, time):
		return self.x.get_tilt(time)

# Function: Traces a circle in the yz plane with x at a specified distance away
# Inputs: Radius of circle, total time of the trace, optional: distance away from head
# Returns: Minimum Jerk piecewise function for the circle
def circle_yz(radius, time, distance = 1.0):
	accuracy = 16.0
	x = []
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0.0, 0.0, 0.0))
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0.0, 0.0, time))

	y = []
	z = []

	y.append(single.Waypoint(-1.0 * radius, 0.0, 0.0, 0.0))
	z.append(single.Waypoint(0.0 + hk.Head_Kinematics().l1, 0.0, 0.0, 0.0))

	for i in range(1, int(accuracy)):
		theta = np.pi*float(i)/(accuracy/2.0)
		y.append(single.Waypoint(-1.0 * radius * np.cos(theta), radius * np.sin(theta), radius * np.cos(theta), time/accuracy ))
		z.append(single.Waypoint(radius * np.sin(theta)+hk.Head_Kinematics().l1, radius*np.cos(theta), -1*radius*np.sin(theta), time/accuracy))

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
		x.append(single.Waypoint(-1 * radius * np.cos(theta) + (hk.Head_Kinematics().l2+distance) , radius * np.sin(theta), radius * np.cos(theta), time/accuracy ))
		z.append(single.Waypoint(radius*np.sin(theta)+hk.Head_Kinematics().l1, radius*np.cos(theta), -1*radius*np.sin(theta), time/accuracy))
	x.append(single.Waypoint(-1 * radius * np.cos(2*np.pi) + (hk.Head_Kinematics().l2+distance) , 0, 0, time/accuracy))
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
def test_script():
	head_min_jerk = None
	eyes_min_jerk = None
	time = 15.0
	y_dist = .55
	z_dist = .6
	head_x = []
	head_x.append(single.Waypoint(1.2, 0, 0, 0))
	head_x.append(single.Waypoint(1.2, 0, 0, time))
	
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(y_dist, 0, 0, time/5))
	head_y.append(single.Waypoint(0, 0, 0, time/5))
	head_y.append(single.Waypoint(-y_dist, 0, 0, time/5))
	head_y.append(single.Waypoint(0, 0, 0, time/5))
	head_y.append(single.Waypoint(y_dist, 0, 0, time/5))
	
	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1+z_dist, 0, 0, 0))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time/5))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1-z_dist, 0, 0, time/5))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time/5))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1+z_dist, 0, 0, time/5))
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time/5))

	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	head_min = Coordinates_3D(x_coord, y_coord, z_coord)

	eyes_x = []
	eyes_y = []
	eyes_z = []
	eyes_x.append(single.Waypoint(1.0, 0, 0, 0))
	eyes_x.append(single.Waypoint(1.0, 0, 0, time))
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_y.append(single.Waypoint(0, 0, 0, time))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, time))

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
def surprised_no(gaze_length = 1.0):
	head_min_jerk = None
	eyes_min_jerk = None
	time = 7.5
	no_distance = .3
	# 40% of maxmimum joint movement
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
	eyes_x.append(single.Waypoint(gaze_length*2 +hk.Head_Kinematics().l2, 0, 0, 0))
	eyes_x.append(single.Waypoint(gaze_length*2 +hk.Head_Kinematics().l2, 0, 0, time))
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
# Inputs: None
# Returns: head and eye min_jerk
# Notes:	Assumes a home configuration
#			TODO: Parabolas are weird
#			Motion order:
#						 1. Initial gaze point forward 
#						2. Tilt head slightly while looking up to the right
#						 3. Head still while eyes roll in parabola form: -(t-.3)**2 + l1
#						 4. Return to Initial gaze point
def roll_eyes():
	head_min_jerk = None
	eyes_min_jerk = None
	time1 = 1.8
	accuracy = 30

	head_x = []
	head_x.append(single.Waypoint(1.0 + hk.Head_Kinematics().l2, 0, 0, 0))
	head_x.append(single.Waypoint(1.0 + hk.Head_Kinematics().l2, 0, 0, time1, False, np.pi/12.0))
	
	head_y = []
	head_y.append(single.Waypoint(0, 0, 0, 0))
	head_y.append(single.Waypoint(-.3, 0, 0, time1))

	head_z = []
	head_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	head_z.append(single.Waypoint(.3 + hk.Head_Kinematics().l1, 0, 0, time1))


	x_coord = single.MinimumJerk(head_x)
	y_coord = single.MinimumJerk(head_y)
	z_coord = single.MinimumJerk(head_z)

	roll_eyes_head_min = Coordinates_3D(x_coord, y_coord, z_coord)
	

	eyes_x = []
	eyes_x.append(single.Waypoint(1.0 + hk.Head_Kinematics().l2, 0, 0, 0))
	eyes_x.append(single.Waypoint(1.0 + hk.Head_Kinematics().l2, 0, 0, time1))

	eyes_y = []
	eyes_y.append(single.Waypoint(0, 0, 0, 0))
	eyes_y.append(single.Waypoint( -1.0/3.0 , 0, 0, time1-time1/float(accuracy)))
	
	eyes_z = []
	eyes_z.append(single.Waypoint(hk.Head_Kinematics().l1, 0, 0, 0))
	
	for i in range(accuracy):
		t = (i+1.0)/(float(accuracy)*2.0)
		eyes_z.append(single.Waypoint( (0.0-1.0) * (3.0*t - .81333)**2+.8, 0, 0, time1/float(accuracy)))
		

	x_coord = single.MinimumJerk(eyes_x)
	y_coord = single.MinimumJerk(eyes_y)
	z_coord = single.MinimumJerk(eyes_z)

	roll_eyes_eyes_min = Coordinates_3D(x_coord, y_coord, z_coord)

	return roll_eyes_head_min, roll_eyes_eyes_min





# f = open('output.txt', 'w')
'''
coordinate1, coordinate2 = test_script()

t = np.arange(0.0, 30, 0.05)
count = 0
x = []
y = []
z = []
while count < 30:
    x.append(coordinate1.get_position(count)[0])
    y.append(coordinate1.get_position(count)[1])
    z.append(coordinate1.get_position(count)[2])
    count = count + .05

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z)
plt.show()
'''
# scale = 10.0

# ax.scatter(t, x, c='red', s=scale, label='x position', alpha=0.75, edgecolors='none')
# ax.scatter(t, y, c='blue', s=scale, label='y position', alpha=0.75, edgecolors='none')
# ax.scatter(t, z, c='green', s=scale, label='z position', alpha=0.75, edgecolors='none')

# ax.legend()
# ax.grid(True)
# plt.show()



# for i in range(0, ran):
	# f.write(str(coordinate2.get_position(val[i])[0]))
	# f.write('\t')
	# f.write(str(coordinate2.get_position(val[i])[1]))
	# f.write('\t')
	# f.write(str(coordinate2.get_acceleration(val[i])[2]))
	# f.write('\n')

# f.write("x = [")
# for i in range(0, ran):
#	 f.write( str((coordinate2.get_position(val[i]))[0]) )
#	 f.write(", ")
# f.write("]\n")

# f.write("y = [")
# for i in range(0, ran):
#	 f.write( str((coordinate2.get_position(val[i]))[1]) )
#	 f.write(", ")
# f.write("]\n")

# f.write("z = [")
# for i in range(0, ran):
#	 f.write( str((coordinate2.get_position(val[i]))[2]) )
#	 f.write(", ")
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