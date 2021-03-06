import serial
#import getch

increment = 10;

gazeKey = 0xA7
gazeFocus = 128
gazePitch = 128
gazeYaw = 128

ser = serial.Serial('/dev/tty.usbmodem0E21EE91', 115200, parity='N')


def lookCenter():
    global gazePitch, gazeYaw
    gazePitch = 128
    gazeYaw = 128
    print ("center", gazePitch, gazeYaw)

def lookRight():
    global gazeYaw
    gazeYaw += increment
    print ("right", gazePitch, gazeYaw)

def lookLeft():
    global gazeYaw
    gazeYaw -= increment
#    print "left", gazePitch, gazeYaw

def lookUp():
    global gazePitch
    gazePitch -= increment
#    print "up", gazePitch, gazeYaw

def lookDown():
    global gazePitch
    gazePitch += increment
#    print "down", gazePitch, gazeYaw

def lookForward():
    global gazePitch, gazeYaw
    gazePitch = 128
    gazeYaw = 128
#    print "forward", gazePitch, gazeYaw

def lookBackward():
    global gazePitch, gazeYaw
    gazePitch = 128
    gazeYaw = 128
#    print "backward", gazePitch, gazeYaw

while True:
    # key = ord(getch.getch())
    # if key == 27: #ESC
    #     break
    # elif key == 32: #spacebar
    #     lookCenter();
    # elif key == 224: #Special keys (arrows, f keys, ins, del, etc.)
    #     key = ord(getch.getch())
    #     if key == 80: #Down arrow
    #         lookDown()
    #     elif key == 77: # Right arrow
    #         lookRight()
    #     elif key == 75: # Left arrow
    #         lookLeft();
    #     elif key == 72: #Up arrow
    #         lookUp()
    
    lookLeft();
    if gazePitch > 255:
        gazePitch = 255
    elif gazePitch < 0:
        gazePitch = 0
    if gazeYaw > 255:
        gazeYaw = 255
    elif gazeYaw < 0:
        gazeYaw = 0

    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)

#    print (newMessage)
#    serialcmd = input(newMessage)
#    ser.write(serialcmd.encode())

