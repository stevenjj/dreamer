import min_jerk_single as single
import numpy as np
import math

class Coordinates():
	def __init__(self, x, y, z, sys = 'c'):
		'''
		if(sys == 's'): 
			self.x = x * math.sin(z) * math.cos(y)
			self.y = x * math.sin(z) * math.sin(y)
			self.z = x * math.cos(z)
		'''
		self.x = x
		self.y = y
		self.z = z

	def get_position(self, time):
		return np.array([self.x.get_position(time), self.y.get_position(time), self.z.get_position(time)])

f = open('output.txt', 'w')
x = []
x.append(single.Waypoint(1, 0, 0, 0))
x.append(single.Waypoint(.75, -.25, 0, 1))
x.append(single.Waypoint(.25, -.25, 0, 1))
x.append(single.Waypoint(0, 0, 0, 1))


y = []
y.append(single.Waypoint(0, 0, 0, 0))
y.append(single.Waypoint(.25, .25, 0, 1))
y.append(single.Waypoint(.75, .25, 0, 1))
y.append(single.Waypoint(1, 0, 0, 1))


z = []
z.append(single.Waypoint(0, 0, 0, 0))
z.append(single.Waypoint(.5, 0, 0, 1))
z.append(single.Waypoint(0, 0, 0, 1))
z.append(single.Waypoint(.5, 0, 0, 1))

'''
x = []
x.append(single.Waypoint(1, 0, 0, 0))
x.append(single.Waypoint(0, -.05, 0, 1))
x.append(single.Waypoint(0, .05, 0, 1))
x.append(single.Waypoint(1, -.05, 0, 1))
x.append(single.Waypoint(1, .05, 0, 1))

y = []
y.append(single.Waypoint(0, 0, 0, 0))
y.append(single.Waypoint(1, -.05, 0, 1))
y.append(single.Waypoint(1, .05, 0, 1))
y.append(single.Waypoint(0, -.05, 0, 1))
y.append(single.Waypoint(0, .05, 0, 1))

z = []
z.append(single.Waypoint(0, 0, 0, 0))
z.append(single.Waypoint(0, .05, 0, 1))
z.append(single.Waypoint(1, -.05, 0, 1))
z.append(single.Waypoint(1, .05, 0, 1))
z.append(single.Waypoint(0, -.05, 0, 1))
'''
x_coord = single.MinimumJerk(x)
y_coord = single.MinimumJerk(y)
z_coord = single.MinimumJerk(z)

coordinate1 = Coordinates(x_coord, y_coord, z_coord)

val = np.arange(0, 7, 0.1)

f.write("x = [")
for i in range(0, 70):
	f.write( str((coordinate1.get_position(val[i]))[0]) )
	f.write(", ")
f.write("]\n")

f.write("y = [")
for i in range(0, 70):
	f.write( str((coordinate1.get_position(val[i]))[1]) )
	f.write(", ")
f.write("]\n")

f.write("z = [")
for i in range(0, 70):
	f.write( str((coordinate1.get_position(val[i]))[2]) )
	f.write(", ")
f.write("]")

'''
	f.write('\t')
	f.write('\t')
'''