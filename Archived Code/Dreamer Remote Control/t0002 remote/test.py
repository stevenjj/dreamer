from msvcrt import getch

while True:
	key = ord(getch())
	if key == 27:
		break
	elif key == 32:
		print 1
	else:
		print 0