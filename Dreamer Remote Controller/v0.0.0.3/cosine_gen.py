from math import cos

timestep = 0.25
ctrlFreq = 550
pi = 3.141592653589

length = int(ctrlFreq*timestep)

for x in range(0, length):
	print int((cos(x*180/length*pi/180)/2+0.5)*1000),","
