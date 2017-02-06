import serial
from msvcrt import getch

gazeKey = 0xA7
gazeFocus = 128
gazePitch = 128
gazeYaw = 128

ser = serial.Serial('COM12', 115200, parity='N')

def lookCenter():
    gazePitch = 128
    gazeYaw = 128
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "center", gazePitch, gazeYaw

def lookRight():
    gazePitch = 128
    gazeYaw = 160
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "right", gazePitch, gazeYaw

def lookLeft():
    gazePitch = 128
    gazeYaw = 96
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "left", gazePitch, gazeYaw

def lookUp():
    gazePitch = 64
    gazeYaw = 128
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "up", gazePitch, gazeYaw

def lookDown():
    gazePitch = 192
    gazeYaw = 128
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "down", gazePitch, gazeYaw

def lookForward():
    gazePitch = 128
    gazeYaw = 128
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "forward", gazePitch, gazeYaw

def lookBackward():
    gazePitch = 128
    gazeYaw = 128
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print "backward", gazePitch, gazeYaw

while True:
    key = ord(getch())
    if key == 27: #ESC
        break
    elif key == 32: #spacebar
        lookCenter();
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
