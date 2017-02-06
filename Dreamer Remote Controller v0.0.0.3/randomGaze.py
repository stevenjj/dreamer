import random
import time
import serial

gazeKey = 0xA7
gazeFocus = 128
gazePitch = 128
gazeYaw = 128

ser = serial.Serial('COM14', 115200, parity='N')

while 1:
    gazeFocus = random.randint(-64,64)+128
    gazePitch = random.randint(-64,64)+128
    gazeYaw = random.randint(-64,64)+128
    newCommand = gazeKey*0x1000000 + gazeFocus*0x10000 + gazePitch*0x100 + gazeYaw
    newMessage = str(newCommand) + "\r"
    ser.write(newMessage)
    print gazeFocus, gazePitch, gazeYaw

    loopWait = random.randint(0,40)/10+2
    time.sleep(loopWait)