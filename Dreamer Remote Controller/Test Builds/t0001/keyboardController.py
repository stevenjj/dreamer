import time
import serial
from msvcrt import getch

ser = serial.Serial('COM12', 115200, parity='N')

def lookCenter():
    print "center"
    cmdCenter = "2810216576\r"
    ser.write(cmdCenter)

def lookRight():
    print "right"
    cmdRight = "2810216703\r"
    ser.write(cmdRight)

def lookLeft():
    print "left"
    cmdLeft = "2810216448\r"
    ser.write(cmdLeft)

def lookUp():
    print "up"
    cmdUp = "2810183808\r"
    ser.write(cmdUp)

def lookDown():
    print "down"
    cmdDown = "2810249088\r"
    ser.write(cmdDown)

def lookForward():
    print "forward"
    cmdForward = "2801827968\r"
    ser.write(cmdForward)

def lookBackward():
    print "backward"
    cmdBackward = "2818539648\r"
    ser.write(cmdBackward)

while True:
    key = ord(getch())
    if key == 27: #ESC
        break
    elif key == 13: #Enter
        select()
    elif key == 224: #Special keys (arrows, f keys, ins, del, etc.)
        key = ord(getch())
        if key == 80: #Down arrow
            lookDown()
        elif key == 77: # Right arrow
            lookRight()
        elif key == 75: # Left arrow
            lookLeft();
        elif key == 72: #Up arrow
            lookUp()