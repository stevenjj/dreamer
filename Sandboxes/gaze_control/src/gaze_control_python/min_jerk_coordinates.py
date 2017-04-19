import min_jerk_single as single
import numpy as np

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

f = open('output.txt', 'w')


x = []
x.append(single.Waypoint(1, 0, 0, 0))
x.append(single.Waypoint(1, 0, 0, 1))
x.append(single.Waypoint(1, 0, 0, 1))
x.append(single.Waypoint(1, 0, 0, 1))
x.append(single.Waypoint(1, 0, 0, 1))

y = []
y.append(single.Waypoint(-.95, 0, 0, 0))
y.append(single.Waypoint(.95, .05, -.05, 1))
y.append(single.Waypoint(.95, .05, .05, 1))
y.append(single.Waypoint(-.95, .05, -.05, 1))
y.append(single.Waypoint(-.95, 0, 0, 1))

z = []
z.append(single.Waypoint(.05, 0, 0, 0))
z.append(single.Waypoint(.05, .05, .05, 1))
z.append(single.Waypoint(.95, .05, -.05, 1))
z.append(single.Waypoint(.95, .05, -.05, 1))
z.append(single.Waypoint(.05, 0, 0, 1))

x_coord = single.MinimumJerk(x)
y_coord = single.MinimumJerk(y)
z_coord = single.MinimumJerk(z)

coordinate1 = Coordinates_3D(x_coord, y_coord, z_coord)


val = np.arange(0, 4, 0.01)
ran = 400
f.write("x = [")
for i in range(0, ran):
	f.write( str((coordinate1.get_position(val[i]))[0]) )
	f.write(", ")
f.write("]\n")

f.write("y = [")
for i in range(0, ran):
	f.write( str((coordinate1.get_position(val[i]))[1]) )
	f.write(", ")
f.write("]\n")

f.write("z = [")
for i in range(0, ran):
	f.write( str((coordinate1.get_position(val[i]))[2]) )
	f.write(", ")
f.write("]")

# '''
# 	f.write('\t')
# 	f.write('\t')
# '''