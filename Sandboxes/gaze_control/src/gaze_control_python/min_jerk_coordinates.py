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

	#In theory all runtimes are the same
	def total_run_time(self):
		return self.x.total_run_time()



def circle(radius, time, distance = 2):
	accuracy = 16.0
	x = []
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, 0))
	for i in range(0, int(accuracy)):
	    x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, time/accuracy))

	y = []
	y.append(single.Waypoint(-1 * radius, 0, 0, 0))
	for i in range(1, int(accuracy)):
		y.append(single.Waypoint(-1 * radius * np.cos(np.pi*i/(accuracy/2)), radius * np.sin(np.pi*i/(accuracy/2)), radius * np.cos(np.pi*i/(accuracy/2)), time/accuracy ))
	y.append(single.Waypoint(-1 * radius * np.cos(2*np.pi), 0, 0, time/8.0 ))
	
	z = []
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, 0))
	for i in range(1, int(accuracy)):
		z.append(single.Waypoint(radius*np.sin(np.pi*i/(accuracy/2))+hk.Head_Kinematics().l1, radius*np.cos(np.pi*i/(accuracy/2)), -1*radius*np.sin(np.pi*i/(accuracy/2)), time/accuracy))
	z.append(single.Waypoint(0+hk.Head_Kinematics().l1, 0, 0, time/accuracy))
	
	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)

	circle_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return circle_min

def clover(radius, time, distance = 2):
	x = []
	x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, 0))
	for i in range(0, 16):
	    x.append(single.Waypoint(distance+hk.Head_Kinematics().l2, 0, 0, time/16.0))

	y = []
	y.append(single.Waypoint(0, 0, 0, 0))
	for i in range(1, 16):
		y.append(single.Waypoint(radius*np.sin(2*np.pi*i/8.0)*np.cos(np.pi*i/8.0), radius*.5*(np.cos(np.pi*i/8.0) + 3*np.cos(np.pi*i/8.0)),  radius*-.5*(1*np.sin(np.pi*i/8.0)-9*np.sin(3*np.pi*i/8.0)), time/16.0))
	y.append(single.Waypoint(0, 0, 0, time/16.0))

	z = []
	z.append(single.Waypoint(0, 0, 0, 0))
	for i in range(1, 16):
		z.append(single.Waypoint(radius*np.sin(2*np.pi*i/8.0)*np.sin(np.pi*i/8.0), radius*.5*(3*np.sin(3*np.pi*i/8.0)-np.sin(np.pi*i/8.0)), radius*.5*(9*np.cos(3*np.pi*i/8.0)-np.cos(np.pi*i/8.0)), time/16.0))
	z.append(single.Waypoint(0, 0, 0, time/16.0))

	x_coord = single.MinimumJerk(x)
	y_coord = single.MinimumJerk(y)
	z_coord = single.MinimumJerk(z)
	clover_min = Coordinates_3D(x_coord, y_coord, z_coord)
	return clover_min

#clover(.15, 4)

# f = open('output.txt', 'w')


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


# val = np.arange(0, 4, 0.01)
# ran = 400
# f.write("x = [")
# for i in range(0, ran):
# 	f.write( str((coordinate1.get_position(val[i]))[0]) )
# 	f.write(", ")
# f.write("]\n")

# f.write("y = [")
# for i in range(0, ran):
# 	f.write( str((coordinate1.get_position(val[i]))[1]) )
# 	f.write(", ")
# f.write("]\n")

# f.write("z = [")
# for i in range(0, ran):
# 	f.write( str((coordinate1.get_position(val[i]))[2]) )
# 	f.write(", ")
# f.write("]")

# '''
# 	f.write('\t')
# 	f.write('\t')
# '''